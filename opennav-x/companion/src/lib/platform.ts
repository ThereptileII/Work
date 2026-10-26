// Native services through Capacitor. In the browser preview the same calls fall
// back to harmless web equivalents, so the app logic is identical everywhere.
import { Capacitor } from '@capacitor/core';
import { App } from '@capacitor/app';
import { Browser } from '@capacitor/browser';
import { Device } from '@capacitor/device';
import { Preferences } from '@capacitor/preferences';
import { PushNotifications } from '@capacitor/push-notifications';
import { SecureStorage } from '@aparajita/capacitor-secure-storage';
import { PUSH_PLATFORMS } from './config.ts';

export const platform = (): 'android' | 'ios' | 'web' => Capacitor.getPlatform() as 'android' | 'ios' | 'web';
export const native = () => Capacitor.isNativePlatform();

const TOKEN_KEY = 'skager.companion.token';
const PENDING_KEY = 'skager.companion.signin';

/** Phone token: iOS Keychain / Android Keystore-backed storage, never synced to iCloud. */
export const secrets = {
  async token(): Promise<string | null> {
    try { const value = await SecureStorage.get(TOKEN_KEY, false, false); return typeof value === 'string' ? value : null; }
    catch { return null; }
  },
  async setToken(value: string): Promise<void> { await SecureStorage.set(TOKEN_KEY, value, false, false); },
  async clearToken(): Promise<void> { await SecureStorage.remove(TOKEN_KEY, false).catch(() => false); },
  /** The PKCE verifier lives only while the browser sign-in is open. */
  async pending(): Promise<any> { try { return await SecureStorage.get(PENDING_KEY, false, false); } catch { return null; } },
  async setPending(value: unknown): Promise<void> { await SecureStorage.set(PENDING_KEY, value as any, false, false); },
  async clearPending(): Promise<void> { await SecureStorage.remove(PENDING_KEY, false).catch(() => false); },
};

/** Last answers from SKAGER Cloud, shown with their time when the phone is offline. */
export const cache = {
  async read<T>(key: string): Promise<T | null> {
    try { const { value } = await Preferences.get({ key: `cache.${key}` }); return value ? JSON.parse(value) as T : null; }
    catch { return null; }
  },
  async write(key: string, value: unknown): Promise<void> {
    await Preferences.set({ key: `cache.${key}`, value: JSON.stringify(value) }).catch(() => {});
  },
  async clear(): Promise<void> { await Preferences.clear().catch(() => {}); },
};

export async function phoneName(): Promise<string> {
  try {
    const info = await Device.getInfo();
    const name = (info.name || info.model || '').replace(/[\u0000-\u001f\u007f]/g, '').trim();
    return name.slice(0, 60) || (platform() === 'ios' ? 'iPhone' : 'Android phone');
  } catch { return 'Phone'; }
}

export async function openSignIn(url: string): Promise<void> {
  if (native()) await Browser.open({ url, presentationStyle: 'popover' });
  else window.location.assign(url);
}
export async function closeSignIn(): Promise<void> { if (native()) await Browser.close().catch(() => {}); }

export function onDeepLink(handler: (url: string) => void): void {
  if (native()) App.addListener('appUrlOpen', event => handler(event.url));
}
export function onForeground(handler: (active: boolean) => void): void {
  if (native()) App.addListener('appStateChange', state => handler(state.isActive));
  document.addEventListener('visibilitychange', () => handler(document.visibilityState === 'visible'));
}

// ---------------------------------------------------------------- notifications
export type PushStatus = 'unconfigured' | 'unsupported' | 'prompt' | 'denied' | 'granted';
export type PushHandlers = {
  token(service: 'fcm' | 'apns', token: string): void;
  received(data: Record<string, string>): void;
  opened(data: Record<string, string>): void;
  failed(): void;
};

export async function pushStatus(): Promise<PushStatus> {
  if (!native()) return 'unsupported';
  if (!PUSH_PLATFORMS.includes(platform())) return 'unconfigured';
  const { receive } = await PushNotifications.checkPermissions();
  return receive === 'granted' ? 'granted' : receive === 'denied' ? 'denied' : 'prompt';
}

let listening = false;
/** Asks once (the OS remembers the answer) and registers with FCM/APNs. */
export async function enablePush(handlers: PushHandlers, ask: boolean): Promise<PushStatus> {
  let status = await pushStatus();
  if (status === 'unconfigured' || status === 'unsupported' || status === 'denied') return status;
  if (status === 'prompt') {
    if (!ask) return status;
    status = (await PushNotifications.requestPermissions()).receive === 'granted' ? 'granted' : 'denied';
    if (status !== 'granted') return status;
  }
  if (!listening) {
    listening = true;
    await PushNotifications.addListener('registration', ({ value }) => handlers.token(platform() === 'ios' ? 'apns' : 'fcm', value));
    await PushNotifications.addListener('registrationError', () => handlers.failed());
    await PushNotifications.addListener('pushNotificationReceived', n => handlers.received((n.data ?? {}) as Record<string, string>));
    await PushNotifications.addListener('pushNotificationActionPerformed', a =>
      handlers.opened((a.notification.data ?? {}) as Record<string, string>));
    if (platform() === 'android') {
      // Must match ANDROID_CHANNELS in web/edge/push.ts.
      await PushNotifications.createChannel({ id: 'skager-alarms', name: 'Alarms', importance: 5, visibility: 1,
        description: 'Anchor alarm, lost contact and critical alerts', vibration: true });
      await PushNotifications.createChannel({ id: 'skager-events', name: 'Boat events', importance: 3, visibility: 1,
        description: 'Alarm ended and contact restored' });
    }
  }
  await PushNotifications.register();
  return status;
}
export async function disablePush(): Promise<void> {
  if (native() && PUSH_PLATFORMS.includes(platform())) await PushNotifications.unregister().catch(() => {});
}

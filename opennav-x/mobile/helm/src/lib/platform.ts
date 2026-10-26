// Native services through Capacitor. In a desktop browser (preview, tests) the
// same calls fall back to web equivalents or do nothing.
import { Capacitor } from '@capacitor/core';
import { App } from '@capacitor/app';
import { Device } from '@capacitor/device';
import { Haptics, ImpactStyle } from '@capacitor/haptics';
import { KeepAwake } from '@capacitor-community/keep-awake';
import { Preferences } from '@capacitor/preferences';
import { StatusBar } from '@capacitor/status-bar';
import { SecureStorage } from '@aparajita/capacitor-secure-storage';
import type { Boat } from './client.ts';
import { keyFromText, keyText } from './client.ts';

export const platform = (): 'android' | 'ios' | 'web' => Capacitor.getPlatform() as 'android' | 'ios' | 'web';
export const native = () => Capacitor.isNativePlatform();

const BOATS = 'skager.helm.boats';
const SETTINGS = 'skager.helm.settings';
const keyName = (serverId: string) => `skager.helm.key.${serverId}`;

export type Settings = { theme: 'auto' | 'day' | 'dusk' | 'night'; keepAwake: boolean; autoConnect: boolean; seenHelp: boolean };
export const DEFAULT_SETTINGS: Settings = { theme: 'auto', keepAwake: true, autoConnect: true, seenHelp: false };

async function readJson<T>(key: string): Promise<T | null> {
  try { const { value } = await Preferences.get({ key }); return value ? JSON.parse(value) as T : null; }
  catch { return null; }
}

/** Paired boats. The device key of each lives in the Keychain/Keystore, never in preferences. */
export const store = {
  async boats(): Promise<Boat[]> { return (await readJson<Boat[]>(BOATS)) ?? []; },
  async saveBoats(boats: Boat[]): Promise<void> { await Preferences.set({ key: BOATS, value: JSON.stringify(boats) }); },
  async key(serverId: string): Promise<Uint8Array | null> {
    try { const v = await SecureStorage.get(keyName(serverId), false, false); return typeof v === 'string' ? keyFromText(v) : null; }
    catch { return null; }
  },
  async setKey(serverId: string, key: Uint8Array): Promise<void> { await SecureStorage.set(keyName(serverId), keyText(key), false, false); },
  async forget(serverId: string): Promise<void> {
    await SecureStorage.remove(keyName(serverId), false).catch(() => false);
    await this.saveBoats((await this.boats()).filter(b => b.serverId !== serverId));
  },
  async settings(): Promise<Settings> { return { ...DEFAULT_SETTINGS, ...(await readJson<Partial<Settings>>(SETTINGS)) }; },
  async saveSettings(s: Settings): Promise<void> { await Preferences.set({ key: SETTINGS, value: JSON.stringify(s) }); },
};

export async function deviceName(): Promise<string> {
  try {
    const info = await Device.getInfo();
    const name = (info.name || info.model || '').replace(/[\u0000-\u001f\u007f]/g, '').trim();
    return name.slice(0, 60) || (platform() === 'ios' ? 'iPhone' : 'Android phone');
  } catch { return 'Phone'; }
}

export async function keepAwake(on: boolean): Promise<void> {
  try { if (on) await KeepAwake.keepAwake(); else await KeepAwake.allowSleep(); } catch { /* unsupported */ }
}

/** Full screen while the SKAGER screen is shown. */
export async function immersive(on: boolean): Promise<void> {
  if (!native()) return;
  try { if (on) await StatusBar.hide(); else await StatusBar.show(); } catch { /* ignore */ }
}

export function tick(): void {
  if (native()) Haptics.impact({ style: ImpactStyle.Medium }).catch(() => {});
}

export function onDeepLink(handler: (url: string) => void): void {
  if (!native()) return;
  App.addListener('appUrlOpen', event => handler(event.url));
  App.getLaunchUrl().then(launch => { if (launch?.url) handler(launch.url); }).catch(() => {});
}

export function onForeground(handler: (active: boolean) => void): void {
  if (native()) App.addListener('appStateChange', state => handler(state.isActive));
  else document.addEventListener('visibilitychange', () => handler(document.visibilityState === 'visible'));
}

export function onBack(handler: () => void): void {
  if (native()) App.addListener('backButton', handler);
}

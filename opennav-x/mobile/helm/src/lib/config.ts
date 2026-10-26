declare const __HELM_VERSION__: string;
declare const __HELM_PREVIEW__: boolean;
export const APP_VERSION: string = typeof __HELM_VERSION__ === 'string' ? __HELM_VERSION__ : 'dev';
/** Browser preview and end-to-end tests only: allows #pair=<link> in the address. */
export const PREVIEW: boolean = typeof __HELM_PREVIEW__ === 'boolean' ? __HELM_PREVIEW__ : false;
/** Reconnect delays after a lost connection, in ms; the last repeats. */
export const RECONNECT_DELAYS = [600, 1500, 3000, 5000, 8000];

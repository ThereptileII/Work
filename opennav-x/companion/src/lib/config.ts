// Build-time constants. esbuild replaces the __SKAGER_*__ identifiers; tests and
// the browser preview fall back to the production values below.
declare const __SKAGER_API__: string | undefined;
declare const __SKAGER_VERSION__: string | undefined;
declare const __SKAGER_PUSH__: string[] | undefined;
declare const __SKAGER_PREVIEW__: boolean | undefined;

/** SKAGER Cloud. The phone only ever talks to this origin (and map tiles). */
export const API_ORIGIN: string = typeof __SKAGER_API__ === 'string' ? __SKAGER_API__ : 'https://beta.skager.app';
export const APP_VERSION: string = typeof __SKAGER_VERSION__ === 'string' ? __SKAGER_VERSION__ : '0.0.0';
/** Platforms whose native project has push set up (Firebase for Android, APNs for iOS).
 * Elsewhere the app never registers, because registering without it can crash the app. */
export const PUSH_PLATFORMS: string[] = typeof __SKAGER_PUSH__ === 'object' && Array.isArray(__SKAGER_PUSH__) ? __SKAGER_PUSH__ : [];
/** Browser preview against a local server only; never set in a phone build. */
export const PREVIEW: boolean = typeof __SKAGER_PREVIEW__ === 'boolean' ? __SKAGER_PREVIEW__ : false;
/** The deep-link scheme the sign-in page returns to (also the application id). */
export const APP_SCHEME = 'app.skager.companion';

/** SKAGER uploads every 15 s; the app asks as often, and only while it is on screen. */
export const POLL_BOAT_MS = 15_000;
export const POLL_LIST_MS = 60_000;
/** Map tiles: OpenFreeMap (OpenMapTiles schema, no key, no tracking cookies). */
export const MAP_TILES = 'https://tiles.openfreemap.org/planet';
export const MAP_GLYPHS = 'https://tiles.openfreemap.org/fonts/{fontstack}/{range}.pbf';

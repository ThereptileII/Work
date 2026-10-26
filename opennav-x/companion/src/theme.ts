// SKAGER design tokens (docs/design/prototype/design-tokens.json palettes). The
// day/dusk/night palettes are the prototype's; night lowers luminance for use
// on board after dark.
export type ThemeName = 'day' | 'dusk' | 'night';
export type ThemeChoice = 'auto' | ThemeName;
export type Palette = { background: string; surface: string; surfaceRaised: string; border: string; text: string;
  secondary: string; muted: string; active: string; instrument: string; advisory: string; critical: string; ais: string;
  water: string; land: string; shore: string; chartText: string; contour: string; route: string; chartSurface: string };

export const PALETTES: Record<ThemeName, Palette> = {
  day: { background: '#152326', surface: '#1d2d31', surfaceRaised: '#26393d', border: '#35464a', text: '#f3f5ee',
    secondary: '#aabdbd', muted: '#7e9699', active: '#b6efce', instrument: '#7bc8d7', advisory: '#ecc48c', critical: '#ec8f87',
    ais: '#cd8dac', water: '#d5e5e5', land: '#eeeee2', shore: '#afbfae', chartText: '#687b7a', contour: '#adcbce',
    route: '#267c76', chartSurface: '#f7f8f0' },
  dusk: { background: '#1d282e', surface: '#25343b', surfaceRaised: '#30444b', border: '#405059', text: '#e2e5db',
    secondary: '#acb9b7', muted: '#819394', active: '#9bc5b1', instrument: '#82b0c1', advisory: '#cfac84', critical: '#ec8f87',
    ais: '#cd8dac', water: '#344f59', land: '#4e615d', shore: '#748779', chartText: '#adbbb1', contour: '#567880',
    route: '#b0dfc8', chartSurface: '#243a40' },
  night: { background: '#0c1115', surface: '#141c21', surfaceRaised: '#1d282f', border: '#29353b', text: '#b8b5a7',
    secondary: '#91988e', muted: '#747d77', active: '#85a995', instrument: '#78989c', advisory: '#aa9170', critical: '#b77569',
    ais: '#a77f8a', water: '#121e24', land: '#25342f', shore: '#46574a', chartText: '#758579', contour: '#2a4149',
    route: '#91bca2', chartSurface: '#152129' },
};

/** Auto follows the phone: light appearance → Day, dark → Night. */
export function resolveTheme(choice: ThemeChoice, prefersDark: boolean): ThemeName {
  return choice === 'auto' ? (prefersDark ? 'night' : 'day') : choice;
}

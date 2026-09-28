import fs from 'node:fs';
import {assemble} from './build-lib.mjs';
import {loadSymbolLibrary} from './symbol-library.mjs';
fs.writeFileSync('index.html', assemble());
fs.writeFileSync('src/symbol-catalogue.json',JSON.stringify(loadSymbolLibrary(),null,2)+'\n');
console.log('Built standalone index.html — no external dependencies.');

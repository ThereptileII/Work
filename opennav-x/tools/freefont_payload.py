"""Pinned, offline GNU FreeFont source and notice bytes; no legal-clearance verdict."""
import hashlib

NOTICE_ROOT = 'docs/third-party/GNU-FreeFont-20120503/'
PACKAGE_ROOT = 'docs/licenses/GNU-FreeFont-20120503/'
SOURCE_FILES = {'docs/third-party/GNU-FreeFont-20120503/AUTHORS': '1baa0cba3f6004ac1277a25c79ab5efb45011102f9ef2ab3a5b7d262088a1f5a',
 'docs/third-party/GNU-FreeFont-20120503/COPYING': '8ceb4b9ee5adedde47b31e975c1d90c73ad27b6b165a1dcd80c7c545eb65b903',
 'docs/third-party/GNU-FreeFont-20120503/CREDITS': 'da8e9145591c82a4e1aa2aeb5bfccd18d61ca3c62efb2e403598c85dbafbe8f5',
 'docs/third-party/GNU-FreeFont-20120503/NOTICE.txt': '3e935c5bc1dbf28bbda06b5c328d03b1d34782a7f2a5c6bcbe5096aff6289e11',
 'docs/third-party/GNU-FreeFont-20120503/README': 'd7375e3982a2e0afc471f8900c46ac6ea41b1112de1f5cdb9ca5c820adf25a33',
 'vendor/freefont-20120503/FreeSans.otf': 'd3e9138ccf76ef516cc6af867e4e7ed8776331947235824f924c5886186b0c80',
 'vendor/freefont-20120503/REBUILD.txt': '2bf0ddef24d54962328a633e1a95edcca63bdb91f028d0db8019780061f47def',
 'vendor/freefont-20120503/freefont-src-20120503.tar.gz': '0d62e2e22655f841c3ac5fb8c8059a52fe2c09c695d9e8eda9903406e942657a',
 'vendor/freefont-20120503/sfnconv/ttf2sfn.c': '7886cb314de625e06f1071ccead43a20aac956b2ef883eb6d5dbc587763b82c4',
 'vendor/freefont-20120503/ssfn.h': 'f2e179248ba1a313da821fcb8181828a7164871846e2863113548e2bdbb213d2'}
NOTICE_FILES = tuple(name for name in SOURCE_FILES if name.startswith(NOTICE_ROOT))


def verify_sources(read):
    """Read repository-relative paths through the caller's trusted inventory."""
    for name, expected in SOURCE_FILES.items():
        try:
            data = read(name)
        except (KeyError, FileNotFoundError) as error:
            raise ValueError('Missing GNU FreeFont source/notice: ' + name) from error
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError('Pinned GNU FreeFont source/notice differs: ' + name)

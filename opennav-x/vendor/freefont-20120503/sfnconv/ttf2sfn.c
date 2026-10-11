/*
 * sfnconv/ttf2sfn.c
 *
 * Copyright (C) 2019 bzt (bztsrc@gitlab)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * @brief PS Type 1, TrueType and OpenType to Scalable Screen Font converter
 *
 * Note: this tools handles memory allocation in a lazy fashion, but
 * that's okay as it's a command line tool which exists immediately
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define SSFN_NOIMPLEMENTATION
#include "../ssfn.h"
#if HAS_ZLIB
#include <zlib.h>
#endif

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H
#include FT_IMAGE_H
#include FT_BBOX_H
#include FT_SFNT_NAMES_H

#define iswhitespace(x) ((x)==0x20||(x)==0xA0)

/*** contour command ***/
typedef struct {
    int type;
    int px;
    int py;
    int c1x;
    int c1y;
    int c2x;
    int c2y;
} cont_t;

/*** glyph fragment ***/
typedef struct {
    int type;
    int pos;
    int len;
    cont_t *data;
    int prod;
} frag_t;

/*** character ***/
typedef struct {
    int unicode;
    int w;
    int h;
    int adv_x;
    int adv_y;
    int bear_l;
    int bear_t;
    int len;
    int *frags;
    int kgrp;
    int klen;
    int *kern;
} char_t;

/*** kerning groups ***/
typedef struct {
    int pos;
    int klen;
    int *kern;
} kgrp_t;

/*** variables ***/
FT_Library library;
FT_Face face;
FT_SfntName name;
FT_GlyphSlot glyph;
int fragments_num = 0, characters_num = 0;
int hinting = 0, zip = 0, adv = 0, gridsize = 1024, unicode, kgrpnum = 0, hints[2048];
int hintunit, frag_len = 0, frag_mx, frag_my, frag_ix, frag_iy, frag_ax, frag_ay;
int min_y, max_y, min_x, max_x, max_s, d_x, d_y, ox, oy, lx, ly, gx, gy;
cont_t *contour = NULL;
frag_t *frags = NULL;
char_t *chars = NULL;
kgrp_t *kgrps = NULL;
ssfn_font_t ssfn_hdr;
char *ssfn_strings;

/**
 * Add hint fragment
 */
void hint_add()
{
    char_t *c;
    int i, j, k, l, m, n, *hintgrid;

    if(!hinting || !characters_num) return;
    for(i=n=0;i<1024;i++) { if(hints[i] >= gridsize/SSFN_HINTING_THRESHOLD) n++; }

    if(n < 2) return;
    c = &chars[characters_num-1];

    hintgrid = (int*)malloc(n * sizeof(int));
    if(!hintgrid) { fprintf(stderr,"memory allocation error\n"); exit(2); }

    for(i=j=l=m=k=0;i<gridsize;i++) {
        if(hints[i] >= gridsize/SSFN_HINTING_THRESHOLD) {
            if(!j) k = i;
            if(i - j > hintunit) {
                hintgrid[l++] = i - j;
                if(m < i - j) m = i - j;
            }
            j = i;
        }
    }
    k++;
    if(!l) {
        free(hintgrid);
        return;
    }
    ssfn_hdr.features |= SSFN_FEAT_HASHINT;
    if(m > 255) ssfn_hdr.features |= SSFN_FEAT_HBIGCRD;

    for(j=-1,i=0;i<fragments_num;i++) {
        if(frags[i].type == SSFN_FRAG_HINTING && frags[i].len == l &&
            !memcmp(frags[i].data, hintgrid, l*sizeof(int))) {
            j = i;
            break;
        }
    }
    if(j==-1) {
        j = fragments_num++;
        frags = (frag_t*)realloc(frags, fragments_num*sizeof(frag_t));
        if(!frags) { fprintf(stderr,"memory allocation error\n"); exit(2); }
        frags[j].type = SSFN_FRAG_HINTING;
        frags[j].len = l;
        frags[j].data = (cont_t*)hintgrid;
    }

    i=c->len*3; c->len++;
    if(c->len>127) {
        fprintf(stderr, "Too many glyph fragments in U+%06x character's glyph.\n", unicode);
        exit(3);
    }
    c->frags = (int*)realloc(c->frags, (i+3)*sizeof(int));
    if(!c->frags) { fprintf(stderr,"memory allocation error\n"); exit(2); }
    /* hinting fragments must come first */
    for(;i; i--) c->frags[i+2] = c->frags[i-1];
    c->frags[0] = k;
    c->frags[1] = c->bear_t;
    c->frags[2] = j;
}

/**
 * Add a fragment to the current character and create character if necessary
 */
void char_add(int x, int y, int w, int h, int f)
{
    char_t *c;
    int i;

    if(!characters_num || chars[characters_num-1].unicode != unicode) {
        characters_num++;
        chars = (char_t*)realloc(chars, characters_num*sizeof(char_t));
        if(!chars) { fprintf(stderr,"memory allocation error\n"); exit(2); }
        c = &chars[characters_num-1];
        c->kgrp = -1;
        c->frags = c->kern = NULL;
        c->len = c->klen = 0;
        c->bear_l = c->bear_t = 65536;
        c->unicode = unicode;
        c->w = c->h = 0;
        c->adv_x = (((glyph->advance.x*(gridsize-1+(gridsize>>7))+(max_s/2))/max_s)+63)/64 + adv; /* make sure we round it up */
        c->adv_y = (((glyph->advance.y*(gridsize-1+(gridsize>>7))+(max_s/2))/max_s)+63)/64;
    } else
        c = &chars[characters_num-1];
    if(w > c->w) c->w = w;
    if(h > c->h) c->h = h;
    if(x < c->bear_l) c->bear_l = x;
    if(y < c->bear_t) c->bear_t = y;
    if(f>=0) {
        i=c->len*3; c->len++;
        if(c->len>127) {
            fprintf(stderr, "Too many glyph fragments in U+%06x character's glyph.\n", unicode);
            exit(3);
        }
        c->frags = (int*)realloc(c->frags, (i+3)*sizeof(int));
        if(!c->frags) { fprintf(stderr,"memory allocation error\n"); exit(2); }
        c->frags[i+0] = x;
        c->frags[i+1] = y;
        c->frags[i+2] = f;
    }
}

/**
 * Add a kerning group
 */
int kgrp_add(int klen, int *kern)
{
    int i;
    for(i=0;i<kgrpnum;i++) {
        if(kgrps[i].klen==klen && !memcmp(kgrps[i].kern, kern, klen*3*sizeof(int))) {
            return i;
        }
    }
    i = kgrpnum++;
    kgrps = (kgrp_t*)realloc(kgrps, kgrpnum*sizeof(frag_t));
    if(!kgrps) { fprintf(stderr,"memory allocation error\n"); exit(2); }
    kgrps[i].pos = 0;
    kgrps[i].klen = klen;
    kgrps[i].kern = kern;
    return i;
}

/**
 * Add a line to a glyph's approximation
 */
void line_add(int x, int y)
{
    if(frag_ix < x) frag_ix = x;
    if(frag_iy < y) frag_iy = y;
    if(frag_ax > x) frag_ax = x;
    if(frag_ay > y) frag_ay = y;
    if(x < min_x) min_x = x;
    if(x > max_x) max_x = x;
    if(y < min_y) min_y = y;
    if(y > max_y) max_y = y;
    gx = x; gy = y;
}

/**
 * Add a curve to a glyph's approximation
 */
void curve_add(int x0,int y0, int x1,int y1, int x2,int y2, int x3,int y3, int l)
{
    int m0x, m0y, m1x, m1y, m2x, m2y, m3x, m3y, m4x, m4y,m5x, m5y;
    if(l<8 && (x0!=x3 || y0!=y3) && x0 > 0 && y0 > 0) {
        m0x = ((x1-x0)/2) + x0;     m0y = ((y1-y0)/2) + y0;
        m1x = ((x2-x1)/2) + x1;     m1y = ((y2-y1)/2) + y1;
        m2x = ((x3-x2)/2) + x2;     m2y = ((y3-y2)/2) + y2;
        m3x = ((m1x-m0x)/2) + m0x;  m3y = ((m1y-m0y)/2) + m0y;
        m4x = ((m2x-m1x)/2) + m1x;  m4y = ((m2y-m1y)/2) + m1y;
        m5x = ((m4x-m3x)/2) + m3x;  m5y = ((m4y-m3y)/2) + m3y;
        curve_add(x0,y0, m0x,m0y, m3x,m3y, m5x,m5y, l+1);
        curve_add(m5x,m5y, m4x,m4y, m2x,m2y, x3,y3, l+1);
    }
    line_add(x3, y3);
}

/**
 * Add a contour command to fragment
 */
void cont_add(int type, int px, int py, int c1x, int c1y, int c2x, int c2y)
{
    int cx, cy, a;
    cont_t *c;

    if(frag_len>32767) {
        fprintf(stderr, "Too many points in contour in U+%06x character's glyph.\n", unicode);
        exit(3);
    }
    if(type!=SSFN_CONTOUR_COLOR) {
        if(px >= gridsize) px = gridsize - 1;   /* due to rounding errors */
        if(py >= gridsize) py = gridsize - 1;
    }
    if(lx>=0 && ly>=0 && px>=0 && py>=0) {
        if(type==SSFN_CONTOUR_CUBIC) {
            cx = ((c1x-lx)<<1)+lx;
            cy = ((c1y-ly)<<1)+ly;
            if(((c2x-cx)<<1)+cx == px && ((c2y-cy)<<1)+cy == py) {
                type = SSFN_CONTOUR_QUAD;
                c1x = cx;
                c1y = cy;
            }
        }
        if(type == SSFN_CONTOUR_LINE) {
            cx = (lx < px) ? px - lx : lx - px;
            cy = (ly < py) ? py - ly : ly - py;
            a = (lx+px) >> 1;
            if(cx < 2) hints[!hints[a] && a && hints[a-1]? a - 1 : a] += cy;
            a = (ly+py) >> 1;
            if(cy < 2) hints[1024+(!hints[1024+a] && a && hints[1024+a-1]? a - 1 : a)] += cx;
        }
    }
    contour = (cont_t*)realloc(contour, (frag_len+1)*sizeof(cont_t));
    if(!contour) { fprintf(stderr,"memory allocation error\n"); exit(2); }
    c = &contour[frag_len++];
    memset(c, 0, sizeof(cont_t));
    c->type = type;
    if(type == SSFN_CONTOUR_COLOR) {
        ssfn_hdr.features |= SSFN_FEAT_HASCMAP;
        c->px = 0; /* todo: cpal_add(px, py, c1x); */
    } else {
        c->px = px; if(px < frag_mx) frag_mx = px;
        c->py = py; if(py < frag_my) frag_my = py;
        if(type > 1) {
            c->c1x = c1x; if(c1x < frag_mx) frag_mx = c1x;
            c->c1y = c1y; if(c1y < frag_my) frag_my = c1y;
            if(type > 2) {
                c->c2x = c2x; if(c2x < frag_mx) frag_mx = c2x;
                c->c2y = c2y; if(c2y < frag_my) frag_my = c2y;
            }
        }
    }
    switch(type) {
        case SSFN_CONTOUR_MOVE:
        case SSFN_CONTOUR_LINE: line_add(px, py); break;
        case SSFN_CONTOUR_QUAD:
            curve_add(gx,gy, ((c1x-gx)>>1)+gx,((c1y-gy)>>1)+gy, ((px-c1x)>>1)+c1x, ((py-c1y)>>1)+c1y, px, py, 0);
        break;
        case SSFN_CONTOUR_CUBIC: curve_add(gx,gy, c1x, c1y, c2x, c2y, px, py, 0); break;
    }
    lx = px; ly = py;
}

/**
 * Add fragment to fragments list
 */
void frag_add()
{
    int i, j=-1, w=0, h=0;

    if(frag_len) {
        for(i=0;i<frag_len;i++) {
            if(contour[i].px > w) w=contour[i].px;
            if(contour[i].py > h) h=contour[i].py;
            contour[i].px -= frag_mx;
            contour[i].py -= frag_my;
            if(contour[i].type > 1) {
                contour[i].c1x -= frag_mx;
                contour[i].c1y -= frag_my;
                if(contour[i].type > 2) {
                    contour[i].c2x -= frag_mx;
                    contour[i].c2y -= frag_my;
                }
            }
        }
        for(i=0;i<fragments_num;i++) {
            if(frags[i].type == SSFN_FRAG_CONTOUR && frags[i].len == frag_len &&
                !memcmp(frags[i].data, contour, frag_len*sizeof(cont_t))) {
                j = i;
                break;
            }
        }
        if(j==-1) {
            j = fragments_num++;
            frags = (frag_t*)realloc(frags, fragments_num*sizeof(frag_t));
            if(!frags) { fprintf(stderr,"memory allocation error\n"); exit(2); }
            frags[j].type = SSFN_FRAG_CONTOUR;
            frags[j].len = frag_len;
            frags[j].data = contour;
            frags[j].prod = (frag_ax - frag_ix) * (frag_ay - frag_iy);
            contour = NULL;
        }
        char_add(frag_mx, frag_my, w, h, j);
        frag_len = 0;
    }
    if(frag_my < min_y) min_y = frag_my;
    frag_mx = frag_my = frag_ix = frag_iy = gridsize; frag_ax = frag_ay = 0;
    gx = gy = -1;
}

/**
 * Sort fragments by which glyph's approximated product and then scanline
 */
int frgsrt(const void *a, const void *b)
{
    if(frags[((int*)a)[2]].type == SSFN_FRAG_HINTING || (frags[((int*)a)[2]].type == SSFN_FRAG_CONTOUR &&
        frags[((int*)b)[2]].type != SSFN_FRAG_HINTING && frags[((int*)b)[2]].type != SSFN_FRAG_CONTOUR)) return -1;
    if(frags[((int*)b)[2]].type == SSFN_FRAG_HINTING) return 1;

    if(frags[((int*)a)[2]].prod == frags[((int*)b)[2]].prod) {
        return (((int*)a)[1] == ((int*)b)[1]) ? (((int*)a)[0] - ((int*)b)[0]) : (((int*)a)[1] - ((int*)b)[1]);
    }

    return frags[((int*)a)[2]].prod - frags[((int*)b)[2]].prod;
}

/*** contour command callbacks for FT2 ***/

int move(const FT_Vector *to, void *ptr __attribute__((unused))) {
    if(frag_len>0) frag_add();
    cont_add(SSFN_CONTOUR_MOVE,
        ((((to->x-ox+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_x,
        ((((oy-to->y+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_y,
        0,0, 0,0);
    return 0;
}

int line(const FT_Vector *to, void *ptr __attribute__((unused))) {
    cont_add(SSFN_CONTOUR_LINE,
        ((((to->x-ox+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_x,
        ((((oy-to->y+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_y,
        0,0, 0,0);
    return 0;
}

int quad(const FT_Vector *control, const FT_Vector *to, void *ptr __attribute__((unused))) {
    cont_add(SSFN_CONTOUR_QUAD,
        ((((to->x-ox+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_x,
        ((((oy-to->y+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_y,
        ((((control->x-ox+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_x,
        ((((oy-control->y+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_y,
        0,0);
    return 0;
}

int cubic(const FT_Vector *control1, const FT_Vector *control2, const FT_Vector *to, void *ptr  __attribute__((unused))) {
    cont_add(SSFN_CONTOUR_CUBIC,
        ((((to->x-ox+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_x,
        ((((oy-to->y+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_y,
        ((((control1->x-ox+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_x,
        ((((oy-control1->y+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_y,
        ((((control2->x-ox+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_x,
        ((((oy-control2->y+32)/64)*(gridsize-1)+(max_s/2))/max_s)+d_y);
    return 0;
}

FT_Outline_Funcs funcs = { move, line, quad, cubic, 0, 0 };

/**
 * Main procedure
 */
int main(int argc, char **argv)
{
    FT_Error error;
    FT_BBox bbox;
    FT_Vector v;
    int i = 0, j, k, l, m, s = 0, fs = 0, cs = 0, ks = 0, kg = 0, haskern = 0, lastkern = 0, *hint;
    char *infile, *outfile, *arg_name = NULL;
    unsigned char *frg = NULL, *chr = NULL, *kern = NULL, *kgrp = NULL;
    long int origsize;
    FILE *f;
#if HAS_ZLIB
    gzFile g;
#endif

    /* clear and set up SSFN header */
    memset(&ssfn_hdr, 0, sizeof(ssfn_font_t));
    memcpy(&ssfn_hdr.magic, SSFN_MAGIC, 4);
    ssfn_hdr.size = sizeof(ssfn_font_t);
    ssfn_hdr.quality = 4;

    /* parse flags */
    while(argv[i+1] && argv[i+1][0] == '-') {
        if(argv[i+1][1]=='a') { i++; adv = atoi(argv[i+1]); i++; continue; }
        if(argv[i+1][1]=='n') { i++; arg_name = argv[i]; i++; continue; }
        for(j=1;argv[i+1][j];j++) {
            switch(argv[i+1][j]) {
                case 'r': ssfn_hdr.family = SSFN_FAMILY_SERIF; break;
                case 's': ssfn_hdr.family = SSFN_FAMILY_SANS; break;
                case 'd': ssfn_hdr.family = SSFN_FAMILY_DECOR; break;
                case 'm': ssfn_hdr.family = SSFN_FAMILY_MONOSPACE; break;
                case 'h': ssfn_hdr.family = SSFN_FAMILY_HAND; break;
                case 'g': hinting = 1; break;
                case 'z': zip = 1; break;
                case '6': gridsize = 1024; ssfn_hdr.quality = 6; break;
                case '5': gridsize = 512; ssfn_hdr.quality = 5; break;
                case '4': gridsize = 256; ssfn_hdr.quality = 4; break;
                case '3': gridsize = 128; ssfn_hdr.quality = 3; break;
                case '2': gridsize = 64; ssfn_hdr.quality = 2; break;
                case '1': gridsize = 32; ssfn_hdr.quality = 1; break;
                case '0': gridsize = 16; ssfn_hdr.quality = 0; break;
                default: fprintf(stderr, "unknown flag '%c'\n", argv[i+1][j]); return 1;
            }
        }
        i++;
    }

    /* get input and output file names, print help if not found */
    if(argc<i+2) {
        printf("Scalable Screen Font by bzt Copyright (C) 2019 MIT license\n\n"
               "%s [-0|-1|-2|-3|-4|-5|-6] [-r|-s|-d|-m|-h] [-g] "
#if HAS_ZLIB
               "[-z] "
#endif
               "[-a<p>] [-n <name>] <in> <out>\n\n", argv[0]);
        printf(" -0..6: quality* from 0 (lowest, smallest size) to 6 (higher, larger size)\n"
               " -n:    set font unique name\n"
               " -r:    font family Serif (default, fonts with \"feet\", like Times New Roman)\n"
               " -s:    font family Sans Serif (without \"feet\", like Helvetica)\n"
               " -d:    font family Decorative (like Dingbats or Fraktur)\n"
               " -m:    font family Monospace (like Fixed or GNU unifont)\n"
               " -h:    font family Handwriting-like (like Cursive)\n"
               " -g:    save grid hinting information\n");
        printf(
#if HAS_ZLIB
               " -z:    compress output with gzip\n"
#endif
               " -a:    add a constant to advance (some fonts need it, others don't)\n"
               " in:    input PS Type1, TTF or OTF font filename\n out:   output SSFN filename\n\n"
               "* - using quality less than 3 can cause significant glyph degradation\n");
        return 1;
    }
    infile = argv[i+1];
    outfile = argv[i+2];

    f = fopen(infile,"rb");
    if(!f) { fprintf(stderr,"unable to load %s\n", infile); return 3; }
    fseek(f, 0, SEEK_END);
    origsize = ftell(f);
    fclose(f);

    /* initialize FT2 and load TTF */
    if(FT_Init_FreeType(&library)) { fprintf(stderr, "unable to initialize FT2\n"); return 2; }
    if(FT_New_Face(library, infile, 0, &face)) { fprintf(stderr, "unable to load %s\n", infile); return 3; }
    if(!(face->face_flags & FT_FACE_FLAG_SCALABLE)) { fprintf(stderr, "not a scalable font %s\n", infile); return 3; }
    if((error = FT_Set_Char_Size(face, 0, gridsize*64, 72, 72))) { fprintf(stderr, "%s\n", FT_Error_String(error)); return 3; }

    /* get font style */
    if(face->style_flags & FT_STYLE_FLAG_BOLD) ssfn_hdr.style |= SSFN_STYLE_BOLD;
    if(face->style_flags & FT_STYLE_FLAG_ITALIC) ssfn_hdr.style |= SSFN_STYLE_ITALIC;

    printf("\rLoaded '%s' num_glyphs: %ld, units_per_EM: %d\n", face->family_name, face->num_glyphs, face->units_per_EM);

    /* get maximum character height */
    min_x = min_y = gridsize; max_x = max_y = -gridsize;
    for(fs=-1,i=0;i<0x110000;i++) {
        j = i*100/0x10FFFF;
        if(j/5!=fs/5) { printf("\r(1) 2  3   Measuring... %3d%%", j); fflush(stdout); fs=j; }
        if(FT_Load_Glyph(face, FT_Get_Char_Index(face, i), FT_LOAD_DEFAULT) ||
            (i && !face->glyph->glyph_index) || FT_Outline_Get_BBox(&face->glyph->outline, &bbox)) continue;
        if(bbox.xMin < min_x) min_x = bbox.xMin;
        if(bbox.xMax > max_x) max_x = bbox.xMax;
        if(bbox.yMin < min_y) min_y = bbox.yMin;
        if(bbox.yMax > max_y) max_y = bbox.yMax;
    }
    ox = min_x; oy = max_y;
    d_x = ((max_x-min_x))/64; /* floor */
    d_y = ((max_y-min_y))/64;
    if(d_x > d_y) {
        max_s = d_x;
        d_y = (((max_s - d_y)/2)*(gridsize-1))/max_s;
        d_x = 0;
    } else {
        max_s = d_y;
        d_x = (((max_s - d_x)/2)*(gridsize-1))/max_s;
        d_y = 0;
    }
    ssfn_hdr.baseline = min_y < 0 ? (((max_y+32)/64)*(gridsize-1)+(max_s/2))/max_s+d_y : 0;
    ssfn_hdr.underline = min_y < 0 ? (((max_y-min_y/2+32)/64)*(gridsize-1)+(max_s/2))/max_s+d_y :
        ((face->ascender-face->underline_position)*(gridsize-1)+face->height/2)/face->height;

    /* construct SSFN string table */
    ssfn_strings = malloc(BUFSIZ);
    if(!ssfn_strings) { fprintf(stderr,"memory allocation error\n"); return 2; }
    memset(ssfn_strings, 0, BUFSIZ);

    if(arg_name) {                                                  /* unique font name */
        s = strlen(arg_name);
        memcpy(ssfn_strings, arg_name, s);
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else if(!FT_Get_Sfnt_Name(face, 3, &name)) {
        memcpy(ssfn_strings, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else if(!FT_Get_Sfnt_Name(face, 20, &name)) {
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else s++;

    if(!FT_Get_Sfnt_Name(face, 1, &name)) {                         /* family name */
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else s++;

    if(!FT_Get_Sfnt_Name(face, 2, &name)) {                         /* subfamily name */
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else s++;

    if(!FT_Get_Sfnt_Name(face, 5, &name)) {                         /* version */
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else s++;

    if(!FT_Get_Sfnt_Name(face, 8, &name)) {                         /* manufacturer */
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else if(!FT_Get_Sfnt_Name(face, 9, &name)) {
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else s++;

    if(!FT_Get_Sfnt_Name(face, 0, &name)) {                         /* copyright */
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else if(!FT_Get_Sfnt_Name(face, 7, &name)) {
        memcpy(ssfn_strings + s, name.string, name.string_len);
        s += name.string_len;
        while(ssfn_strings[s-1]==' ' || ssfn_strings[s-1]=='\t') s--;
        ssfn_strings[s++] = 0;
    } else s++;

    ssfn_hdr.size = ssfn_hdr.fragments_offs = sizeof(ssfn_font_t) + s;
    ssfn_hdr.size += 4;

    hintunit = 1 << (ssfn_hdr.quality > 4 ? ssfn_hdr.quality - 4 : 0);

    /* iterate on characters and contour outlines */
    min_x = min_y = gridsize; max_x = max_y = 0;
    for(fs=-1,i=0;i<0x110000;i++) {
        j = i*100/0x10FFFF;
        if(j/5!=fs/5) { printf("\r 1 (2) 3   Compressing glyphs... %3d%%", j); fflush(stdout); fs=j; }
        if(FT_Load_Glyph(face, FT_Get_Char_Index(face, i), FT_LOAD_DEFAULT) ||
            (i && !face->glyph->glyph_index)) continue;

        frag_len = 0; lx = ly = gx = gy = -1;
        frag_mx = frag_my = frag_ix = frag_iy = gridsize; frag_ax = frag_ay = 0;
        unicode = i;
        glyph = face->glyph;
        memset(hints, 0, sizeof(hints));

        error = FT_Outline_Decompose(&face->glyph->outline, &funcs, NULL);
        if(error) { printf("decompose error %x %s\n",error,FT_Error_String(error)); continue; }
        if(iswhitespace(i)) char_add(0,0,0,0,-1); else frag_add();
        hint_add();
    }
    if(contour) free(contour);
    ssfn_hdr.bbox_top = min_y;
    ssfn_hdr.bbox_left = min_x;
    ssfn_hdr.bbox_bottom = max_y;
    ssfn_hdr.bbox_right = max_x;

    /* serialize fragments */
    for(fs=0,i=0;i<fragments_num;i++) {
        frg = (unsigned char*)realloc(frg, fs+16);
        if(!frg) { fprintf(stderr,"memory allocation error\n"); return 2; }
        frags[i].pos = ssfn_hdr.fragments_offs + fs;
        if(frags[i].type == SSFN_FRAG_HINTING) {
            frg = (unsigned char*)realloc(frg, fs+2+2*frags[i].len);
            if(!frg) { fprintf(stderr,"memory allocation error\n"); return 2; }
            if(frags[i].len > 16) {
                frg[fs++] =  (((frags[i].len-1)>>8) & 0xF) | 0xF0;
                frg[fs++] =  (frags[i].len-1) & 0xFF;
            } else
                frg[fs++] =  ((frags[i].len-1) & 0xF) | 0xE0;
            hint = (int*)frags[i].data;
            for(j=0;j<frags[i].len;j++) {
                frg[fs++] = hint[j] & 0xFF;
                if(ssfn_hdr.features & SSFN_FEAT_HBIGCRD)
                    frg[fs++] = (hint[j]>>8) & 0xFF;
            }
        } else {
            if(frags[i].len < 2) continue;
            if(frags[i].len>65)
                frg[fs++] = (((frags[i].len - 2) >> 8) & 0x3F) | 0x40;
            frg[fs++] = (frags[i].len - 2) & 0xFF;
            if(frags[i].data[0].type != SSFN_CONTOUR_MOVE) {
                fprintf(stderr,"contour fragment %d does not start with move to command\n", i);
                return 2;
            }
            /* SSFN_CONTOUR_MOVE */
            if(ssfn_hdr.quality < 5) {
                frg[fs++] = frags[i].data[0].px & 0xFF;
                frg[fs++] = frags[i].data[0].py & 0xFF;
            } else {
                frg[fs++] = (((frags[i].data[0].px >> 8) & 3)) | (((frags[i].data[0].py >> 8) & 3) << 4);
                frg[fs++] = frags[i].data[0].px & 0xFF;
                frg[fs++] = frags[i].data[0].py & 0xFF;
            }
            for(j=1;j<frags[i].len;j++) {
                frg = (unsigned char*)realloc(frg, fs+16);
                if(!frg) { fprintf(stderr,"memory allocation error\n"); return 2; }
                switch(ssfn_hdr.quality) {
                    /* low quality */
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                        switch(frags[i].data[j].type) {
                            case SSFN_CONTOUR_LINE:
                                frg[fs++] = (frags[i].data[j].px & 0x7F) | 0x80;
                                frg[fs++] = frags[i].data[j].py & 0x7F;
                            break;
                            case SSFN_CONTOUR_QUAD:
                                frg[fs++] = frags[i].data[j].px & 0x7F;
                                frg[fs++] = (frags[i].data[j].py & 0x7F) | 0x80;
                                frg[fs++] = frags[i].data[j].c1x & 0x7F;
                                frg[fs++] = frags[i].data[j].c1y & 0x7F;
                            break;
                            case SSFN_CONTOUR_CUBIC:
                                frg[fs++] = (frags[i].data[j].px & 0x7F) | 0x80;
                                frg[fs++] = (frags[i].data[j].py & 0x7F) | 0x80;
                                frg[fs++] = frags[i].data[j].c1x & 0x7F;
                                frg[fs++] = frags[i].data[j].c1y & 0x7F;
                                frg[fs++] = frags[i].data[j].c2x & 0x7F;
                                frg[fs++] = frags[i].data[j].c2y & 0x7F;
                            break;
                            case SSFN_CONTOUR_COLOR:
                                frg[fs++] = frags[i].data[j].px >> 7;
                                frg[fs++] = frags[i].data[j].px & 0x7F;
                            break;
                        }
                    break;

                    /* reasonable quality */
                    case 4:
                        switch(frags[i].data[j].type) {
                            case SSFN_CONTOUR_LINE:
                                frg[fs++] = SSFN_CONTOUR_LINE;
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                            break;
                            case SSFN_CONTOUR_QUAD:
                                frg[fs++] = SSFN_CONTOUR_QUAD;
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                                frg[fs++] = frags[i].data[j].c1x & 0xFF;
                                frg[fs++] = frags[i].data[j].c1y & 0xFF;
                            break;
                            case SSFN_CONTOUR_CUBIC:
                                frg[fs++] = SSFN_CONTOUR_CUBIC;
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                                frg[fs++] = frags[i].data[j].c1x & 0xFF;
                                frg[fs++] = frags[i].data[j].c1y & 0xFF;
                                frg[fs++] = frags[i].data[j].c2x & 0xFF;
                                frg[fs++] = frags[i].data[j].c2y & 0xFF;
                            break;
                            case SSFN_CONTOUR_COLOR:
                                frg[fs++] = 0;
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                            break;
                        }
                    break;

                    /* medium quality */
                    case 5:
                        switch(frags[i].data[j].type) {
                            case SSFN_CONTOUR_LINE:
                                frg[fs++] = SSFN_CONTOUR_LINE
                                    | (((frags[i].data[j].px >> 8) & 1) << 2)
                                    | (((frags[i].data[j].py >> 8) & 1) << 3);
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                            break;
                            case SSFN_CONTOUR_QUAD:
                                frg[fs++] = SSFN_CONTOUR_QUAD
                                    | (((frags[i].data[j].px >> 8) & 1) << 2)
                                    | (((frags[i].data[j].py >> 8) & 1) << 3)
                                    | (((frags[i].data[j].c1x >> 8) & 1) << 4)
                                    | (((frags[i].data[j].c1y >> 8) & 1) << 5);
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                                frg[fs++] = frags[i].data[j].c1x & 0xFF;
                                frg[fs++] = frags[i].data[j].c1y & 0xFF;
                            break;
                            case SSFN_CONTOUR_CUBIC:
                                frg[fs++] = SSFN_CONTOUR_CUBIC
                                    | (((frags[i].data[j].px >> 8) & 1) << 2)
                                    | (((frags[i].data[j].py >> 8) & 1) << 3)
                                    | (((frags[i].data[j].c1x >> 8) & 1) << 4)
                                    | (((frags[i].data[j].c1y >> 8) & 1) << 5)
                                    | (((frags[i].data[j].c2x >> 8) & 1) << 6)
                                    | (((frags[i].data[j].c2y >> 8) & 1) << 7);
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                                frg[fs++] = frags[i].data[j].c1x & 0xFF;
                                frg[fs++] = frags[i].data[j].c1y & 0xFF;
                                frg[fs++] = frags[i].data[j].c2x & 0xFF;
                                frg[fs++] = frags[i].data[j].c2y & 0xFF;
                            break;
                            case SSFN_CONTOUR_COLOR:
                                frg[fs++] = 0;
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                            break;
                        }
                    break;

                    /* high quality */
                    default:
                        switch(frags[i].data[j].type) {
                            case SSFN_CONTOUR_LINE:
                                frg[fs++] = SSFN_CONTOUR_LINE
                                    | (((frags[i].data[j].px >> 8) & 3) << 2)
                                    | (((frags[i].data[j].py >> 8) & 3) << 4);
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                            break;
                            case SSFN_CONTOUR_QUAD:
                                frg[fs++] = SSFN_CONTOUR_QUAD
                                    | (((frags[i].data[j].px >> 8) & 3) << 2)
                                    | (((frags[i].data[j].py >> 8) & 3) << 4);
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                                frg[fs++] =
                                      (((frags[i].data[j].c1x >> 8) & 3) << 0)
                                    | (((frags[i].data[j].c1y >> 8) & 3) << 2);
                                frg[fs++] = frags[i].data[j].c1x & 0xFF;
                                frg[fs++] = frags[i].data[j].c1y & 0xFF;
                            break;
                            case SSFN_CONTOUR_CUBIC:
                                frg[fs++] = SSFN_CONTOUR_CUBIC
                                    | (((frags[i].data[j].px >> 8) & 3) << 2)
                                    | (((frags[i].data[j].py >> 8) & 3) << 4);
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                                frg[fs++] = frags[i].data[j].py & 0xFF;
                                frg[fs++] =
                                      (((frags[i].data[j].c1x >> 8) & 3) << 0)
                                    | (((frags[i].data[j].c1y >> 8) & 3) << 2)
                                    | (((frags[i].data[j].c2x >> 8) & 3) << 4)
                                    | (((frags[i].data[j].c2y >> 8) & 3) << 6);
                                frg[fs++] = frags[i].data[j].c1x & 0xFF;
                                frg[fs++] = frags[i].data[j].c1y & 0xFF;
                                frg[fs++] = frags[i].data[j].c2x & 0xFF;
                                frg[fs++] = frags[i].data[j].c2y & 0xFF;
                            break;
                            case SSFN_CONTOUR_COLOR:
                                frg[fs++] = 0;
                                frg[fs++] = frags[i].data[j].px & 0xFF;
                            break;
                        }
                    break;
                }
            }
        }
        if(frags[i].data) free(frags[i].data);
    }
    ssfn_hdr.characters_offs[0] = ssfn_hdr.fragments_offs + fs;
    ssfn_hdr.size += fs;

    /* serialize character map */
    if(ssfn_hdr.quality < 5 && ssfn_hdr.characters_offs[0] < 65536) l = 0; else
    if(ssfn_hdr.characters_offs[0] < 1048576) l = 1; else l = 2;
    min_x = max_x = lastkern = haskern = 0;
    unicode = -1;
    for(i=ks=0;i<characters_num;i++) {
        j = chars[i].unicode - unicode - 1;
        chr = (unsigned char*)realloc(chr, cs+256+chars[i].len*10);
        if(!chr) { fprintf(stderr,"memory allocation error\n"); return 2; }
        while(j > 0) {
            if(j <= 64) {
                chr[cs++] = ((j-1) & 0x3F) | 0x80;
                break;
            } else {
                while(j>16384) {
                    chr[cs++] = 0xFF;
                    chr[cs++] = 0xFF;
                    j -= 16384;
                }
                if(j > 64) {
                    chr[cs++] = (((j-1) >> 8) & 0x3F) | 0xC0;
                    chr[cs++] = (j-1) & 0xFF;
                    break;
                }
            }
        }
        chr[cs++] = chars[i].len & 0x7F;
        chr[cs++] = (((chars[i].h >> 8) & 15) << 4) | ((chars[i].w >> 8) & 15);
        chr[cs++] = (((chars[i].adv_y >> 8) & 15) << 4) | ((chars[i].adv_x >> 8) & 15);
        chr[cs++] = (((chars[i].bear_t >> 8) & 15) << 4) | ((chars[i].bear_l >> 8) & 15);
        chr[cs++] = chars[i].w & 0xFF;
        chr[cs++] = chars[i].h & 0xFF;
        chr[cs++] = chars[i].adv_x & 0xFF;
        chr[cs++] = chars[i].adv_y & 0xFF;
        chr[cs++] = chars[i].bear_l & 0xFF;
        chr[cs++] = chars[i].bear_t & 0xFF;
        /* stupid TTF allows out of order fragments, let's try to sort them properly */
        qsort(chars[i].frags, chars[i].len, 3*sizeof(int), frgsrt);
        for(j=0;j<chars[i].len;j++) {
            chars[i].frags[j*3+0] -= chars[i].bear_l;
            chars[i].frags[j*3+1] -= chars[i].bear_t;
            switch(l) {
                case 0:
                    chr[cs++] = frags[chars[i].frags[j*3+2]].pos & 0xFF;
                    chr[cs++] = (frags[chars[i].frags[j*3+2]].pos >> 8) & 0xFF;
                    chr[cs++] = chars[i].frags[j*3+0] & 0xFF;
                    chr[cs++] = chars[i].frags[j*3+1] & 0xFF;
                break;

                case 1:
                    chr[cs++] = frags[chars[i].frags[j*3+2]].pos & 0xFF;
                    chr[cs++] = (frags[chars[i].frags[j*3+2]].pos >> 8) & 0xFF;
                    chr[cs++] = ((frags[chars[i].frags[j*3+2]].pos >> 16) & 0xF) |
                        (((chars[i].frags[j*3+0] >> 8) & 3) << 4) |
                        (((chars[i].frags[j*3+1] >> 8) & 3) << 6);
                    chr[cs++] = chars[i].frags[j*3+0] & 0xFF;
                    chr[cs++] = chars[i].frags[j*3+1] & 0xFF;
                break;

                default:
                    chr[cs++] = frags[chars[i].frags[j*3+2]].pos & 0xFF;
                    chr[cs++] = (frags[chars[i].frags[j*3+2]].pos >> 8) & 0xFF;
                    chr[cs++] = ((frags[chars[i].frags[j*3+2]].pos >> 16) & 0xFF);
                    chr[cs++] = ((chars[i].frags[j*3+0] >> 8) & 15) | (((chars[i].frags[j*3+1] >> 8) & 0xF) << 4);
                    chr[cs++] = chars[i].frags[j*3+0] & 0xFF;
                    chr[cs++] = chars[i].frags[j*3+1] & 0xFF;
                break;
            }
        }
        if(chars[i].frags) free(chars[i].frags);
        /* add to kerning */
        j = i*100/characters_num;
        if(j/5!=ks/5) {
            if(!ks) printf("\r                                      ");
            printf("\r 1  2 (3)  Kerning... %3d%%", j); fflush(stdout); ks=j;
        }
        if(chars[i].unicode>32)
            for(j=0;j<characters_num;j++) {
                if(chars[j].unicode<33) continue;
                FT_Get_Kerning(face, chars[i].unicode, chars[j].unicode, 0, &v);
                v.x = (((v.x*(gridsize-1)+(max_s/2))/max_s)+63)/64;
                v.y = (((v.y*(gridsize-1)+(max_s/2))/max_s)+63)/64;
                /* skip if scaling to 2/3 of maximum rasterizer resolution gives 0 */
                if( !(((v.x << (16 - (4+ssfn_hdr.quality))) * 192 + (1 << 16) - 1) >> 16) &&
                    !(((v.y << (16 - (4+ssfn_hdr.quality))) * 192 + (1 << 16) - 1) >> 16)) continue;
                if(chars[i].klen>32767) {
                    fprintf(stderr,"Too many kerning pairs for U+%06x, truncated to 32768\n", chars[i].unicode);
                    break;
                }
                haskern++;
                if(chars[j].unicode > lastkern) lastkern = chars[j].unicode;
                for(k=m=0;k<chars[i].klen;k+=3) {
                    if(chars[i].kern[k]==chars[j].unicode) { m=1; break; }
                }
                if(!m) {
                    k = chars[i].klen*3;
                    chars[i].klen++;
                    chars[i].kern = (int*)realloc(chars[i].kern, (k+3)*sizeof(int));
                    if(!chars[i].kern) { fprintf(stderr,"memory allocation error\n"); return 2; }
                    chars[i].kern[k+0] = chars[j].unicode;
                }
                chars[i].kern[k+1] = v.x;
                chars[i].kern[k+2] = v.y;
                if(v.x < min_x) min_x = v.x;
                if(v.y < min_x) min_x = v.y;
                if(v.x > max_x) max_x = v.x;
                if(v.y > max_x) max_x = v.y;
            }
        unicode = chars[i].unicode;
    }
    j = 0x110000 - unicode;
    chr = (unsigned char*)realloc(chr, cs+256);
    if(!chr) { fprintf(stderr,"memory allocation error\n"); return 2; }
    while(j > 0) {
        if(j <= 64) {
            chr[cs++] = ((j-1) & 0x3F) | 0x80;
            break;
        } else {
            while(j>16384) {
                chr[cs++] = 0xFF;
                chr[cs++] = 0xFF;
                j -= 16384;
            }
            if(j > 64) {
                chr[cs++] = (((j-1) >> 8) & 0x3F) | 0xC0;
                chr[cs++] = (j-1) & 0xFF;
                break;
            }
        }
    }
    ssfn_hdr.size += cs;

    printf("\rFragments: %d, Characters: %d, bbox: (%d,%d) (%d,%d), baseline: %d, underline: %d\n",
        fragments_num, characters_num, ssfn_hdr.bbox_left,ssfn_hdr.bbox_top,
        ssfn_hdr.bbox_right,ssfn_hdr.bbox_bottom, ssfn_hdr.baseline, ssfn_hdr.underline);

    /* serialize kerning information */
    if(haskern) {
        ssfn_hdr.kerning_offs = ssfn_hdr.characters_offs[0] + cs;
        for(kgrpnum=i=0;i<characters_num;i++) {
            if(chars[i].klen) chars[i].kgrp = kgrp_add(chars[i].klen, chars[i].kern);
        }
        printf("Kerning: %d pairs, %d groups\n", haskern, kgrpnum);

        if(min_x < -128 || max_x > 127) ssfn_hdr.features |= SSFN_FEAT_KBIGCRD;
        if(lastkern > 32767) ssfn_hdr.features |= SSFN_FEAT_KBIGCHR;

        for(i=0;i<kgrpnum;i++) {
            for(k=j=0;j<kgrps[i].klen;j++) {
                if(kgrps[i].kern[j*3+1]) k++;
                if(kgrps[i].kern[j*3+2]) k++;
            }
            kgrp = (unsigned char*)realloc(kgrp, kg+k*5);
            if(!kgrp) { fprintf(stderr,"memory allocation error\n"); return 2; }
            kgrps[i].pos = kg;
            if(k<=128) {
                kgrp[kg++] = ((k - 1) & 0x7F);
            } else {
                kgrp[kg++] = (((k - 1) >> 8) & 0x7F) | 0x80;
                kgrp[kg++] = (k - 1) & 0xFF;
            }
            for(j=0;j<kgrps[i].klen;j++) {
                if(kgrps[i].kern[j*3+1]) {
                    kgrp[kg++] = kgrps[i].kern[j*3] & 0xFF;
                    if(ssfn_hdr.features & SSFN_FEAT_KBIGCHR) {
                        kgrp[kg++] = (kgrps[i].kern[j*3] >> 8) & 0xFF;
                        kgrp[kg++] = (kgrps[i].kern[j*3] >> 16) & 0x7F;
                    } else
                        kgrp[kg++] = (kgrps[i].kern[j*3] >> 8) & 0x7F;
                    kgrp[kg++] = kgrps[i].kern[j*3+1] & 0xFF;
                    if(ssfn_hdr.features & SSFN_FEAT_KBIGCRD)
                        kgrp[kg++] = (kgrps[i].kern[j*3+1] >> 8) & 0xFF;
                }
                if(kgrps[i].kern[j*3+2]) {
                    kgrp[kg++] = kgrps[i].kern[j*3] & 0xFF;
                    if(ssfn_hdr.features & SSFN_FEAT_KBIGCHR) {
                        kgrp[kg++] = (kgrps[i].kern[j*3] >> 8) & 0xFF;
                        kgrp[kg++] = ((kgrps[i].kern[j*3] >> 16) & 0x7F) | 0x80;
                    } else
                        kgrp[kg++] = ((kgrps[i].kern[j*3] >> 8) & 0x7F) | 0x80;
                    kgrp[kg++] = kgrps[i].kern[j*3+2] & 0xFF;
                    if(ssfn_hdr.features & SSFN_FEAT_KBIGCRD)
                        kgrp[kg++] = (kgrps[i].kern[j*3+2] >> 8) & 0xFF;
                }
            }
        }
        unicode = -1;
        for(ks=k=i=0;i<characters_num;i++) {
            if(chars[i].kgrp == -1) continue;
            j = chars[i].unicode - unicode - 1;
            while(j > 0) {
                if(j <= 64) { ks++; break;
                } else {
                    while(j>16384) { ks += 2; j -= 16384; }
                    if(j > 64) { ks += 2; break; }
                }
            }
            for(j=0;j<characters_num-i && j<128 &&
                chars[i].unicode+j == chars[i+j].unicode && chars[i].kgrp == chars[i+j].kgrp;j++);
            j--;
            unicode = chars[i].unicode + j;
            i += j;
            k++;
        }
        j = 0x110000 - unicode;
        while(j > 0) {
            if(j <= 64) { ks++; break;
            } else {
                while(j>16384) { ks += 2; j -= 16384; }
                if(j > 64) { ks += 2; break; }
            }
        }
        if(ks+k*3+kg > 65535) {
            ssfn_hdr.features |= SSFN_FEAT_KBIGLKP;
            k *= 4; k += ks;
        } else { k *= 3; k += ks; }

        unicode = -1;
        for(ks=i=0;i<characters_num;i++) {
            if(chars[i].kgrp == -1) continue;
            j = chars[i].unicode - unicode - 1;
            kern = (unsigned char*)realloc(kern, ks+256+2);
            if(!kern) { fprintf(stderr,"memory allocation error\n"); return 2; }
            while(j > 0) {
                if(j <= 64) {
                    kern[ks++] = ((j-1) & 0x3F) | 0x80;
                    break;
                } else {
                    while(j>16384) {
                        kern[ks++] = 0xFF;
                        kern[ks++] = 0xFF;
                        j -= 16384;
                    }
                    if(j > 64) {
                        kern[ks++] = (((j-1) >> 8) & 0x3F) | 0xC0;
                        kern[ks++] = (j-1) & 0xFF;
                        break;
                    }
                }
            }
            for(j=0;j<characters_num-i && j<128 &&
                chars[i].unicode+j == chars[i+j].unicode && chars[i].kgrp == chars[i+j].kgrp;j++);
            j--;
            kern[ks++] = j & 0x7F;
            l = kgrps[chars[i].kgrp].pos + k;
            kern[ks++] = l & 0xFF;
            kern[ks++] = (l >> 8) & 0xFF;
            if(ssfn_hdr.features & SSFN_FEAT_KBIGLKP) {
                kern[ks++] = (l >> 16) & 0xFF;
            }
            unicode = chars[i].unicode + j;
            i += j;
        }
        j = 0x110000 - unicode;
        kern = (unsigned char*)realloc(kern, ks+256);
        if(!kern) { fprintf(stderr,"memory allocation error\n"); return 2; }
        while(j > 0) {
            if(j <= 64) {
                kern[ks++] = ((j-1) & 0x3F) | 0x80;
                break;
            } else {
                while(j>16384) {
                    kern[ks++] = 0xFF;
                    kern[ks++] = 0xFF;
                    j -= 16384;
                }
                if(j > 64) {
                    kern[ks++] = (((j-1) >> 8) & 0x3F) | 0xC0;
                    kern[ks++] = (j-1) & 0xFF;
                    break;
                }
            }
        }
    } else {
        printf("No kerning information\n");
        ssfn_hdr.kerning_offs = 0;
    }
    free(frags);
    free(chars);

    ssfn_hdr.size += ks + kg;

    /* write output file */
#if HAS_ZLIB
    if(zip) {
        g = gzopen(outfile, "wb");
        if(!g) { fprintf(stderr, "unable to write %s\n", outfile); return 4; }
        gzwrite(g, &ssfn_hdr, sizeof(ssfn_font_t));
        gzwrite(g, ssfn_strings, s);
        gzwrite(g, frg, fs);
        gzwrite(g, chr, cs);
        gzwrite(g, kern, ks);
        gzwrite(g, kgrp, kg);
        gzwrite(g, SSFN_ENDMAGIC, 4);
        gzclose(g);
        f = fopen(outfile, "rb");
        if(f) {
            fseek(f, 0, SEEK_END);
            s = (int)ftell(f);
            fclose(f);
        } else
            s = 0;
    } else
#endif
    {
        f = fopen(outfile, "wb");
        if(!f) { fprintf(stderr, "unable to write %s\n", outfile); return 4; }
        fwrite(&ssfn_hdr, sizeof(ssfn_font_t), 1, f);
        fwrite(ssfn_strings, s, 1, f);
        fwrite(frg, fs, 1, f);
        fwrite(chr, cs, 1, f);
        fwrite(kern, ks, 1, f);
        fwrite(kgrp, kg, 1, f);
        fwrite(SSFN_ENDMAGIC, 4, 1, f);
        fclose(f);
        s = ssfn_hdr.size;
    }

    i = 1000 - (s*1000/origsize);
    if(i<0) { infile="+"; i *= -1; } else { infile="-"; }
    printf("Original %ld bytes, compressed %d bytes, %s%d.%d%%\n", origsize, s, infile, i/10, i%10);

    /* free resources */
    free(ssfn_strings);
    free(frg);
    free(chr);
    free(kern);

    return 0;
}

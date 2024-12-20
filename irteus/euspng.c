///////////////////////////////////////////////////////////////////////////////
///
/// $Id: euspng.c $
///
/// Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
///
/// This software is a collection of EusLisp code for robot applications,
/// which has been developed by the JSK Laboratory for the IRT project.
/// For more information on EusLisp and it's application to the robotics,
/// please refer to the following papers.
///
/// Toshihiro Matsui
/// Multithread object-oriented language euslisp for parallel and
///  asynchronous programming in robotics
/// Workshop on Concurrent Object-based Systems,
///  IEEE 6th Symposium on Parallel and Distributed Processing, 1994
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
/// * Redistributions of source code must retain the above copyright notice,
///   this list of conditions and the following disclaimer.
/// * Redistributions in binary form must reproduce the above copyright notice,
///   this list of conditions and the following disclaimer in the documentation
///   and/or other materials provided with the distribution.
/// * Neither the name of JSK Robotics Laboratory, The University of Tokyo
///   (JSK) nor the names of its contributors may be used to endorse or promote
///   products derived from this software without specific prior written
///   permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
/// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
/// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
/// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
/// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
/// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
/// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
/// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
/// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///

#pragma init (register_euspng)

#include <png.h>
#include "eus.h"

extern pointer ___euspng();
static void register_euspng()
{ add_module_initializer("___euspng", ___euspng);}

pointer PNG_READ_IMAGE(register context *ctx, int n, register pointer *argv)
{
  char *file_name;
  pointer ret, image_ptr;
  ckarg(1);
  if (isstring(argv[0])) file_name = (char *)(argv[0]->c.str.chars);
  else error(E_NOSTRING);

  FILE *fp = fopen(file_name, "rb");
  if (!fp) {
    error(E_OPENFILE);
    return(NIL);
  }

  png_structp png_ptr;
  png_infop info_ptr;
  png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
  info_ptr = png_create_info_struct(png_ptr);

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
    fclose(fp);
    error(E_EOF);
    return(NIL);
  }

  png_init_io(png_ptr, fp);
  png_read_info(png_ptr, info_ptr);
  int width = png_get_image_width(png_ptr, info_ptr);
  int height = png_get_image_height(png_ptr, info_ptr);
  int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
  int channels = png_get_channels(png_ptr, info_ptr);
  int color_type = png_get_color_type(png_ptr, info_ptr);
  //fprintf(stderr, "bit_depth = %d, channels %d, color_type =%d (pal:%d,gray:%d,rgb:%d,rgba:%d)\n", bit_depth, channels, color_type, PNG_COLOR_TYPE_PALETTE,PNG_COLOR_TYPE_GRAY,PNG_COLOR_TYPE_RGB,PNG_COLOR_TYPE_RGB_ALPHA);
  switch (color_type) {
  case PNG_COLOR_TYPE_PALETTE:
    png_set_palette_to_rgb(png_ptr);
    break;
  case PNG_COLOR_TYPE_GRAY:
    #ifdef PNG_READ_RGB_TO_GRAY_SUPPORTED
    if ( bit_depth < 8) png_set_gray_to_rgb(png_ptr);
    #else
    if ( bit_depth < 8) png_set_gray_1_2_4_to_8(png_ptr);
    #endif
    break;
  case PNG_COLOR_TYPE_RGB:
    //png_set_bgr(png_ptr);
    if (bit_depth == 16) png_set_strip_16(png_ptr); // 16bit -> 8bit
    break;
  case PNG_COLOR_TYPE_RGB_ALPHA:
    if (bit_depth == 16) png_set_strip_16(png_ptr); // 16bit -> 8bit
    png_set_invert_alpha(png_ptr);
    //png_set_bgr(png_ptr);
    //png_set_strip_alpha(png_ptr);
    // RGBA -> rgb , GA -> g
    png_color_16 my_background = {0xff, 0xff, 0xff, 0xff, 0xff};
    png_set_background(png_ptr, &my_background,
                       PNG_BACKGROUND_GAMMA_SCREEN, 0, 1.0);

    break;
  }
  png_read_update_info(png_ptr, info_ptr);
  width = png_get_image_width(png_ptr, info_ptr); height = png_get_image_height(png_ptr, info_ptr);;
  bit_depth = png_get_bit_depth(png_ptr, info_ptr); channels = png_get_channels(png_ptr, info_ptr);
  color_type = png_get_color_type(png_ptr, info_ptr);

  png_bytep * row_pointers = (png_bytep *)malloc(height*sizeof(png_bytep));
  int y, byte_per_scanline = png_get_rowbytes(png_ptr, info_ptr);
  image_ptr = makebuffer(height*byte_per_scanline);
  for(y=0;y<height;y++){
    row_pointers[y] = image_ptr->c.str.chars+y*byte_per_scanline;
  }
  png_read_image(png_ptr, row_pointers);
  free(row_pointers);
  png_read_end(png_ptr,info_ptr);
  png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
  fclose(fp);

  ret=cons(ctx,image_ptr,NIL);
  ret=cons(ctx,makeint(channels),ret);
  ret=cons(ctx,makeint(height),ret);
  ret=cons(ctx,makeint(width),ret);
  return (ret);
}

pointer PNG_WRITE_IMAGE(register context *ctx, int n, register pointer *argv)
{
  char *file_name;
  png_bytep image_ptr;
  int width, height, channels;
  pointer bg;
  ckarg2(5,6);
  if (isstring(argv[0])) file_name = (char *)(argv[0]->c.str.chars);
  else error(E_NOSTRING);
  width  = ckintval(argv[1]);
  height = ckintval(argv[2]);
  channels  = ckintval(argv[3]);

  if (n==6 && argv[5]!=NIL) {  /* set background color */
    bg = argv[5];
    if (!isfltvector(bg)) error(E_NOVECTOR);
    if (3!=vecsize(bg)) error(E_VECSIZE);
  }else{
    bg=NIL;
  }

  if(bg==NIL) {
    image_ptr = (png_bytep)(argv[4]->c.str.chars);
  } else {
    int x, y;
    png_byte bg_r=bg->c.fvec.fv[0]*255, bg_g=bg->c.fvec.fv[1]*255, bg_b=bg->c.fvec.fv[2]*255;
    image_ptr = malloc(width*height*4);
    for(y = 0; y < height; y++) {
      for(x = 0; x < width; x++) {
        png_byte r, g, b;
        r = ((png_bytep)(argv[4]->c.str.chars))[(y*width+x)*3+0];
        g = ((png_bytep)(argv[4]->c.str.chars))[(y*width+x)*3+1];
        b = ((png_bytep)(argv[4]->c.str.chars))[(y*width+x)*3+2];
        image_ptr[(y*width+x)*4+0] = r;
        image_ptr[(y*width+x)*4+1] = g;
        image_ptr[(y*width+x)*4+2] = b;
        image_ptr[(y*width+x)*4+3] = ((r==bg_r)&&(g==bg_g)&&(b==bg_b))?0:255;
      }
    }
  }

  FILE *fp = fopen(file_name, "wb");
  if (!fp) {
    error(E_OPENFILE);
    if(bg!=NIL) {free(image_ptr);}
    return(NIL);
  }

  png_structp png_ptr;
  png_infop info_ptr;
  png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
  info_ptr = png_create_info_struct(png_ptr);

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
    if(bg!=NIL) {free(image_ptr);}
    error(E_EOF);
    return(NIL);
  }

  png_init_io(png_ptr, fp);
  png_set_IHDR(png_ptr, info_ptr, width, height, 8, (bg==NIL)?PNG_COLOR_TYPE_RGB:PNG_COLOR_TYPE_RGB_ALPHA, //GRAY
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
  png_bytep * row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
  int y, byte_per_scanline = png_get_rowbytes(png_ptr, info_ptr);
  for(y=0;y<height;y++){
    row_pointers[y] = &(image_ptr[y*byte_per_scanline]);
  }
  png_set_rows(png_ptr, info_ptr, row_pointers);

  png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, (png_voidp)NULL);
  png_write_end(png_ptr, info_ptr);

  free(row_pointers);
  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(fp);

  if(bg!=NIL) {free(image_ptr);}
  return (T);
}

#include "defun.h" // redefine defun for update defun() API
pointer ___euspng(register context *ctx, int n, register pointer *argv)
{
    pointer mod=argv[0];

    defun(ctx, "PNG-READ-IMAGE",  mod, PNG_READ_IMAGE, NULL);
    defun(ctx, "PNG-WRITE-IMAGE", mod, PNG_WRITE_IMAGE, NULL);
}


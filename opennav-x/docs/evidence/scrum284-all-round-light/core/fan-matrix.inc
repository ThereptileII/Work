void ActualMatrix(GLfloat* out,int width,int height,double rotation) { struct VP { int pix_width,pix_height; double rotation; } v{width,height,rotation}; auto* vp=&v;
  mat4x4 m;
  float vp_transform[16];
  mat4x4_identity(m);
  mat4x4_scale_aniso((float(*)[4])vp_transform, m, 2.0 / (float)vp->pix_width,
                     -2.0 / (float)vp->pix_height, 1.0);
  // Rotate
  mat4x4 Q;
  mat4x4_rotate_Z(Q, (float(*)[4])vp_transform, vp->rotation);
  mat4x4_translate_in_place(Q, -vp->pix_width / 2.0, -vp->pix_height / 2.0, 0);

std::copy_n(&Q[0][0],16,out); }

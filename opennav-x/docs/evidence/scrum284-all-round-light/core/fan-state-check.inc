  {
    const GLint position=glGetAttribLocation(S52texture_2D_shader_program,"position"),uv=glGetAttribLocation(S52texture_2D_shader_program,"aUV");
    auto snapshot=[&] {
      std::vector<double> state;
      for(GLenum key:{GL_CURRENT_PROGRAM,GL_ACTIVE_TEXTURE,GL_ARRAY_BUFFER_BINDING,GL_PIXEL_UNPACK_BUFFER_BINDING,GL_UNPACK_ALIGNMENT,GL_UNPACK_ROW_LENGTH,GL_UNPACK_SKIP_ROWS,GL_UNPACK_SKIP_PIXELS,GL_BLEND_SRC_RGB,GL_BLEND_DST_RGB,GL_BLEND_SRC_ALPHA,GL_BLEND_DST_ALPHA,GL_BLEND_EQUATION_RGB,GL_BLEND_EQUATION_ALPHA}) {GLint n;glGetIntegerv(key,&n);state.push_back(n);}
      state.push_back(glIsEnabled(GL_BLEND));state.push_back(glIsEnabled(GL_CULL_FACE));
      GLint active;glGetIntegerv(GL_ACTIVE_TEXTURE,&active);glActiveTexture(GL_TEXTURE0);GLint tex;glGetIntegerv(GL_TEXTURE_BINDING_2D,&tex);state.push_back(tex);
      if(GLEW_VERSION_3_3 || GLEW_ARB_sampler_objects){glGetIntegerv(GL_SAMPLER_BINDING,&tex);state.push_back(tex);}glActiveTexture(active);
      for(GLuint i:{GLuint(position),GLuint(uv)}) {
        for(GLenum key:{GL_VERTEX_ATTRIB_ARRAY_ENABLED,GL_VERTEX_ATTRIB_ARRAY_SIZE,GL_VERTEX_ATTRIB_ARRAY_TYPE,GL_VERTEX_ATTRIB_ARRAY_NORMALIZED,GL_VERTEX_ATTRIB_ARRAY_STRIDE,GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING}) {GLint n;glGetVertexAttribiv(i,key,&n);state.push_back(n);}
        if(GLEW_VERSION_3_0){GLint n;glGetVertexAttribiv(i,GL_VERTEX_ATTRIB_ARRAY_INTEGER,&n);state.push_back(n);}
        void* p;glGetVertexAttribPointerv(i,GL_VERTEX_ATTRIB_ARRAY_POINTER,&p);state.push_back(reinterpret_cast<uintptr_t>(p));
      }
      for(const char* name:{"MVMatrix","TransformMatrix"}) {GLfloat m[16];glGetUniformfv(S52texture_2D_shader_program,glGetUniformLocation(S52texture_2D_shader_program,name),m);state.insert(state.end(),m,m+16);}
      GLint s;glGetUniformiv(S52texture_2D_shader_program,glGetUniformLocation(S52texture_2D_shader_program,"uTex"),&s);state.push_back(s);return state;
    };
    GLuint buffers[2],textures[2],sampler=0;glGenBuffers(2,buffers);glGenTextures(2,textures);
    glBindBuffer(GL_ARRAY_BUFFER,buffers[0]);glBufferData(GL_ARRAY_BUFFER,256,nullptr,GL_STATIC_DRAW);
    glVertexAttribPointer(position,3,GL_SHORT,GL_TRUE,8,reinterpret_cast<void*>(8));glDisableVertexAttribArray(position);
    if(GLEW_VERSION_3_0)glVertexAttribIPointer(uv,2,GL_SHORT,8,reinterpret_cast<void*>(16));
    else glVertexAttribPointer(uv,2,GL_SHORT,GL_FALSE,8,reinterpret_cast<void*>(16));
    glEnableVertexAttribArray(uv);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER,buffers[1]);glBufferData(GL_PIXEL_UNPACK_BUFFER,256,nullptr,GL_STATIC_DRAW);
    glPixelStorei(GL_UNPACK_ALIGNMENT,8);glPixelStorei(GL_UNPACK_ROW_LENGTH,17);glPixelStorei(GL_UNPACK_SKIP_ROWS,2);glPixelStorei(GL_UNPACK_SKIP_PIXELS,3);
    glActiveTexture(GL_TEXTURE0);glBindTexture(GL_TEXTURE_2D,textures[0]);
    if(GLEW_VERSION_3_3 || GLEW_ARB_sampler_objects){glGenSamplers(1,&sampler);glSamplerParameteri(sampler,GL_TEXTURE_MIN_FILTER,GL_NEAREST_MIPMAP_LINEAR);glBindSampler(0,sampler);}
    glActiveTexture(GL_TEXTURE3);glBindTexture(GL_TEXTURE_2D,textures[1]);
    GLfloat matrix[]={2,0,0,0,0,3,0,0,0,0,1,0,4,5,0,1};
    glUseProgram(S52texture_2D_shader_program);
    glUniformMatrix4fv(glGetUniformLocation(S52texture_2D_shader_program,"MVMatrix"),1,GL_FALSE,matrix);
    glUniformMatrix4fv(glGetUniformLocation(S52texture_2D_shader_program,"TransformMatrix"),1,GL_FALSE,matrix);
    glUniform1i(glGetUniformLocation(S52texture_2D_shader_program,"uTex"),3);
    glUseProgram(S52ring_shader_program);glEnable(GL_CULL_FACE);
    glBlendFuncSeparate(GL_ONE,GL_ZERO,GL_DST_ALPHA,GL_ONE_MINUS_DST_ALPHA);glBlendEquationSeparate(GL_FUNC_SUBTRACT,GL_FUNC_REVERSE_SUBTRACT);
    for(bool blend:{false,true}) {
      if(blend)glEnable(GL_BLEND);else glDisable(GL_BLEND);
      auto before=snapshot();Check(([&]{Light light;return lib.RenderCARC_GLSL(&light.node,&light.rule)==1;})(),"hostile actual all-round method state draw");Check(before==snapshot(),"exact touched GL state restored");Check(glGetError()==GL_NO_ERROR,"GL state draw error-free");
    }
    // A translated/subrect viewport cannot be silently interpreted as full view.
    auto before=snapshot();glViewport(7,11,300,310);Check(!DrawCaFanGL(tile,S52texture_2D_shader_program,400,400),"noncanonical viewport refuses");GLint viewport[4];glGetIntegerv(GL_VIEWPORT,viewport);Check(viewport[0]==7 && viewport[1]==11 && viewport[2]==300 && viewport[3]==310,"viewport unchanged");Check(before==snapshot(),"refusal state unchanged");glViewport(0,0,400,400);
    glUseProgram(0);glDisableVertexAttribArray(position);glDisableVertexAttribArray(uv);glBindBuffer(GL_ARRAY_BUFFER,0);glBindBuffer(GL_PIXEL_UNPACK_BUFFER,0);
    glPixelStorei(GL_UNPACK_ALIGNMENT,4);glPixelStorei(GL_UNPACK_ROW_LENGTH,0);glPixelStorei(GL_UNPACK_SKIP_ROWS,0);glPixelStorei(GL_UNPACK_SKIP_PIXELS,0);
    glActiveTexture(GL_TEXTURE0);glBindTexture(GL_TEXTURE_2D,0);if(sampler){glBindSampler(0,0);glDeleteSamplers(1,&sampler);}glActiveTexture(GL_TEXTURE3);glBindTexture(GL_TEXTURE_2D,0);glActiveTexture(GL_TEXTURE0);
    glDeleteTextures(2,textures);glDeleteBuffers(2,buffers);glDisable(GL_CULL_FACE);glDisable(GL_BLEND);
  }

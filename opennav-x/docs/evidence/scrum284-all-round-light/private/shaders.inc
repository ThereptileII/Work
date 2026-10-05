static const GLchar *S52texture_2D_vertex_shader_source =
    "attribute vec2 position;\n"
    "attribute vec2 aUV;\n"
    "uniform mat4 MVMatrix;\n"
    "uniform mat4 TransformMatrix;\n"
    "varying vec2 varCoord;\n"
    "void main() {\n"
    "   gl_Position = MVMatrix * TransformMatrix * vec4(position, 0.0, 1.0);\n"
    "   //varCoord = aUV.st;\n"
    "   varCoord = aUV;\n"
    "}\n";

static const GLchar *S52texture_2D_fragment_shader_source =
    "precision highp float;\n"
    "uniform sampler2D uTex;\n"
    "varying vec2 varCoord;\n"
    "void main() {\n"
    "   gl_FragColor = texture2D(uTex, varCoord);\n"
    "}\n";

static const GLchar *S52ring_vertex_shader_source =
    "precision highp float;\n"
    "attribute vec2 aPos;\n"
    "uniform mat4 MVMatrix;\n"
    "uniform mat4 TransformMatrix;\n"
    "void main() {\n"
    "   gl_Position = MVMatrix * TransformMatrix * vec4(aPos, 0.0, 1.0);\n"
    "}\n";

static const GLchar *S52ring_fragment_shader_source =
    "precision highp float;\n"
    "uniform float ring_width;\n"
    "uniform float circle_radius;\n"
    "uniform vec4 ring_color;\n"
    "uniform vec2 circle_center;\n"
    "uniform float sector_1;\n"
    "uniform float sector_2;\n"

    "void main(){\n"
    "const float PI = 3.14159265358979323846264;\n"
    "bool bdraw = false;\n"

    "float angle = atan(gl_FragCoord.y-circle_center.y, "
    "gl_FragCoord.x-circle_center.x);\n"
    "angle = PI/2.0 - angle;\n"
    "if(angle < 0.0) angle += PI * 2.0;\n"

    "if(sector_2 > PI * 2.0){\n"
    "    if((angle > sector_1) && (angle < (PI * 2.0) )){\n"
    "        bdraw = true;\n"
    "    }\n"
    "    if(angle < sector_2 - (PI * 2.0)){\n"
    "        bdraw = true;\n"
    "    }\n"
    "} else {\n"
    "    if((angle > sector_1) && (angle < sector_2)){\n"
    "        bdraw = true;\n"
    "    }\n"
    "}\n"

    "if(bdraw){\n"
    "   float d = distance(gl_FragCoord.xy, circle_center);\n"
    "   float delta = 0.9;\n"
    "   if (d > circle_radius + ring_width/2) {\n"
    "       discard;\n"
    "   } else if( d > (circle_radius )) {\n"
    "     float alpha = smoothstep(circle_radius + ring_width/2 -delta, circle_radius + ring_width/2, d);\n"
    "     gl_FragColor = ring_color;\n"
    "     gl_FragColor.w = 1-alpha;\n"
    "   }\n"
    "   else if( d > (circle_radius - ring_width/2 - delta)) {\n"
    "       gl_FragColor = ring_color;\n"
    "       float alpha = smoothstep(circle_radius - ring_width/2 - delta, circle_radius - ring_width/2, d);\n"
    "       gl_FragColor.w = alpha;\n"
    "   }\n"
    "   else  {\n"
    "       discard;\n"
    "   }\n"
    "} else{\n"
    "   discard;\n"
    "}\n"
    "}\n";


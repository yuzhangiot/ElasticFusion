#version 330 core

layout (location = 0) in vec4 position;
layout (location = 1) in vec4 color;
layout (location = 2) in vec4 normal;

uniform mat4 MVP;

out vec4 vColor;

void main()
{
    vColor = color;
    gl_Position = MVP * vec4(position.xyz, 1.0);

}
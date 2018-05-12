#version 330 core

layout (location = 0) in vec4 position;
layout (location = 1) in vec4 color;
layout (location = 2) in vec4 normal;

uniform mat4 MVP;
uniform int drawMode;

out vec4 vColor;

void main()
{
	if(drawMode == 1) {
    	vColor = color;
	}
	else {
		vColor = vec4(5.0 / 255.0, 39.0 / 255.0, 175.0 / 255.0, 0.2);
	}
    gl_Position = MVP * vec4(position.xyz, 1.0);

}
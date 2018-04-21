#version 330 core

in vec4 vColor;


out vec4 FragColor;

void main()
{    
  FragColor = vec4(vColor.xyz, 1.0f);  
  // FragColor = vec4(0, 0, 1, 1);
}
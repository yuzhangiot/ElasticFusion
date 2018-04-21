/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#version 330 core

layout (location = 0) in vec4 vPosition;
layout (location = 1) in vec4 vColor;
layout (location = 2) in vec4 vFaces;

out vec4 vPosition0;
out vec4 vColor0;
out vec4 vNormRad0;

uniform vec4 cam; //cx, cy, 1/fx, 1/fy

#include "color.glsl"
#include "surfels.glsl"

void main()
{
    vPosition0 = vec4(vec3(vPosition), 10000);
    // vColor0 = vec4(0,0,0,0);
    vNormRad0 = vec4(0,0,0,0);

    vColor0 = vec4(encodeColor(vec3(vColor)), 0, 0, 1);
    // vNormRad0 = vec4(vec3(vNormRad), getRadius(vPosition.z, vNormRad.z));
}

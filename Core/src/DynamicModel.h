#ifndef DYNAMIC_MODEL_H_
#define DYNAMIC_MODEL_H_

#include "Shaders/Shaders.h"
#include "Shaders/Uniform.h"
#include "Shaders/FeedbackBuffer.h"
#include "GPUTexture.h"
#include "Utils/Resolution.h"
#include "IndexMap.h"
#include "Utils/Stopwatch.h"
#include "Utils/Intrinsics.h"
#include <pangolin/gl/gl.h>
#include <Eigen/LU>

#include "Defines.h"

class DynamicModel
{
public:
	DynamicModel();
	virtual ~DynamicModel();

	void initialise(const FeedbackBuffer & rawFeedback,
                        const FeedbackBuffer & filteredFeedback);

	static const int TEXTURE_DIMENSION;
    static const int MAX_VERTICES;
    static const int NODE_TEXTURE_DIMENSION;
    static const int MAX_NODES;
	

private:
    //First is the vbo, second is the fid
    std::pair<GLuint, GLuint> * vbos;
    int target, renderSource;

    const int bufferSize;

    GLuint countQuery;
    unsigned int count;

    std::shared_ptr<Shader> initProgram;
    std::shared_ptr<Shader> drawProgram;
    std::shared_ptr<Shader> drawSurfelProgram;

    //For supersample fusing
    std::shared_ptr<Shader> dataProgram;
    std::shared_ptr<Shader> updateProgram;
    std::shared_ptr<Shader> unstableProgram;
    pangolin::GlRenderBuffer renderBuffer;

    //We render updated vertices vec3 + confidences to one texture
    GPUTexture updateMapVertsConfs;

    //We render updated colors vec3 + timestamps to another
    GPUTexture updateMapColorsTime;

    //We render updated normals vec3 + radii to another
    GPUTexture updateMapNormsRadii;

    //16 floats stored column-major yo'
    GPUTexture deformationNodes;

    GLuint newUnstableVbo, newUnstableFid;

    pangolin::GlFramebuffer frameBuffer;
    GLuint uvo;
    int uvSize;
};





#endif
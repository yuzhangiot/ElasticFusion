#include "DynamicModel.h"


const int DynamicModel::TEXTURE_DIMENSION = 3072;
const int DynamicModel::MAX_VERTICES = DynamicModel::TEXTURE_DIMENSION * DynamicModel::TEXTURE_DIMENSION;
const int DynamicModel::NODE_TEXTURE_DIMENSION = 16384;
const int DynamicModel::MAX_NODES = DynamicModel::NODE_TEXTURE_DIMENSION / 16; //16 floats per node


DynamicModel::DynamicModel()
:  target(0),
   renderSource(1),
   bufferSize(MAX_VERTICES * Vertex::SIZE),
   count(0),
   initProgram(loadProgramFromFile("init_unstable.vert")),
   drawProgram(loadProgramFromFile("draw_feedback.vert", "draw_feedback.frag")),
   drawSurfelProgram(loadProgramFromFile("draw_global_surface.vert", "draw_global_surface.frag", "draw_global_surface.geom")),
   dataProgram(loadProgramFromFile("data.vert", "data.frag", "data.geom")),
   updateProgram(loadProgramFromFile("update.vert")),
   unstableProgram(loadProgramGeomFromFile("copy_unstable.vert", "copy_unstable.geom")),
   renderBuffer(TEXTURE_DIMENSION, TEXTURE_DIMENSION),
   updateMapVertsConfs(TEXTURE_DIMENSION, TEXTURE_DIMENSION, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT),
   updateMapColorsTime(TEXTURE_DIMENSION, TEXTURE_DIMENSION, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT),
   updateMapNormsRadii(TEXTURE_DIMENSION, TEXTURE_DIMENSION, GL_RGBA32F, GL_LUMINANCE, GL_FLOAT),
   deformationNodes(NODE_TEXTURE_DIMENSION, 1, GL_LUMINANCE32F_ARB, GL_LUMINANCE, GL_FLOAT)
{
    vbos = new std::pair<GLuint, GLuint>[2];

    float * vertices = new float[bufferSize];

    memset(&vertices[0], 0, bufferSize);

    glGenTransformFeedbacks(1, &vbos[0].second);
    glGenBuffers(1, &vbos[0].first);
    glBindBuffer(GL_ARRAY_BUFFER, vbos[0].first);
    glBufferData(GL_ARRAY_BUFFER, bufferSize, &vertices[0], GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0); //unbind

    glGenTransformFeedbacks(1, &vbos[1].second);
    glGenBuffers(1, &vbos[1].first);
    glBindBuffer(GL_ARRAY_BUFFER, vbos[1].first);
    glBufferData(GL_ARRAY_BUFFER, bufferSize, &vertices[0], GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0); //unbind

    delete [] vertices;

    vertices = new float[Resolution::getInstance().numPixels() * Vertex::SIZE];

    memset(&vertices[0], 0, Resolution::getInstance().numPixels() * Vertex::SIZE);

    glGenTransformFeedbacks(1, &newUnstableFid);
    glGenBuffers(1, &newUnstableVbo);
    glBindBuffer(GL_ARRAY_BUFFER, newUnstableVbo);
    glBufferData(GL_ARRAY_BUFFER, Resolution::getInstance().numPixels() * Vertex::SIZE, &vertices[0], GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    delete [] vertices;

    std::vector<Eigen::Vector2f> uv;

    for(int i = 0; i < Resolution::getInstance().width(); i++)
    {
        for(int j = 0; j < Resolution::getInstance().height(); j++)
        {
            uv.push_back(Eigen::Vector2f(((float)i / (float)Resolution::getInstance().width()) + 1.0 / (2 * (float)Resolution::getInstance().width()),
                                   ((float)j / (float)Resolution::getInstance().height()) + 1.0 / (2 * (float)Resolution::getInstance().height())));
        }
    }

    uvSize = uv.size();

    glGenBuffers(1, &uvo);
    glBindBuffer(GL_ARRAY_BUFFER, uvo);
    glBufferData(GL_ARRAY_BUFFER, uvSize * sizeof(Eigen::Vector2f), &uv[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    frameBuffer.AttachColour(*updateMapVertsConfs.texture); // vertex confidence
    frameBuffer.AttachColour(*updateMapColorsTime.texture); // color time
    frameBuffer.AttachColour(*updateMapNormsRadii.texture); // norm radius
    frameBuffer.AttachDepth(renderBuffer);

    updateProgram->Bind();

    int locUpdate[3] =
    {
        glGetVaryingLocationNV(updateProgram->programId(), "vPosition0"),
        glGetVaryingLocationNV(updateProgram->programId(), "vColor0"),
        glGetVaryingLocationNV(updateProgram->programId(), "vNormRad0"),
    };

    glTransformFeedbackVaryingsNV(updateProgram->programId(), 3, locUpdate, GL_INTERLEAVED_ATTRIBS);

    updateProgram->Unbind();

    // VBO
    dataProgram->Bind();

    int dataUpdate[3] =
    {
        glGetVaryingLocationNV(dataProgram->programId(), "vPosition0"),
        glGetVaryingLocationNV(dataProgram->programId(), "vColor0"),
        glGetVaryingLocationNV(dataProgram->programId(), "vNormRad0"),
    };

    glTransformFeedbackVaryingsNV(dataProgram->programId(), 3, dataUpdate, GL_INTERLEAVED_ATTRIBS);

    dataProgram->Unbind();

    unstableProgram->Bind();

    int unstableUpdate[3] =
    {
        glGetVaryingLocationNV(unstableProgram->programId(), "vPosition0"),
        glGetVaryingLocationNV(unstableProgram->programId(), "vColor0"),
        glGetVaryingLocationNV(unstableProgram->programId(), "vNormRad0"),
    };

    glTransformFeedbackVaryingsNV(unstableProgram->programId(), 3, unstableUpdate, GL_INTERLEAVED_ATTRIBS);

    unstableProgram->Unbind();

    initProgram->Bind();

    int locInit[3] =
    {
        glGetVaryingLocationNV(initProgram->programId(), "vPosition0"),
        glGetVaryingLocationNV(initProgram->programId(), "vColor0"),
        glGetVaryingLocationNV(initProgram->programId(), "vNormRad0"),
    };

    glTransformFeedbackVaryingsNV(initProgram->programId(), 3, locInit, GL_INTERLEAVED_ATTRIBS);

    // generate queires
    glGenQueries(1, &countQuery);

    //Empty both transform feedbacks, turn off pixelize
    glEnable(GL_RASTERIZER_DISCARD);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, vbos[0].second); // bind transform buffer

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbos[0].first); // bind buffer to TF

    glBeginTransformFeedback(GL_POINTS);

    glDrawArrays(GL_POINTS, 0, 0); // draw buffer and TF

    glEndTransformFeedback();

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, vbos[1].second);

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbos[1].first);

    glBeginTransformFeedback(GL_POINTS);

    glDrawArrays(GL_POINTS, 0, 0); // draw buffer and TF

    glEndTransformFeedback();

    glDisable(GL_RASTERIZER_DISCARD);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    initProgram->Unbind();
}

DynamicModel::~DynamicModel()
{
	glDeleteBuffers(1, &vbos[0].first);
    glDeleteTransformFeedbacks(1, &vbos[0].second);

    glDeleteBuffers(1, &vbos[1].first);
    glDeleteTransformFeedbacks(1, &vbos[1].second);

    glDeleteQueries(1, &countQuery);

    glDeleteBuffers(1, &uvo);

    glDeleteTransformFeedbacks(1, &newUnstableFid);
    glDeleteBuffers(1, &newUnstableVbo);

    delete [] vbos;
}






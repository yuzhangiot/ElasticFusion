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
   dataProgram(loadProgramFromFile("data_dyn.vert", "data_dyn.frag", "data_dyn.geom")),
   updateProgram(loadProgramGeomFromFile("update_dyn.vert","update_dyn.geom")),
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

void DynamicModel::initialise(const FeedbackBuffer & rawFeedback,
                             const FeedbackBuffer & filteredFeedback)
{
    initProgram->Bind();

    glBindBuffer(GL_ARRAY_BUFFER, rawFeedback.vbo);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, 0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f)));

    glBindBuffer(GL_ARRAY_BUFFER, filteredFeedback.vbo);

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 2));

    glEnable(GL_RASTERIZER_DISCARD);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, vbos[target].second);

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbos[target].first);

    glBeginTransformFeedback(GL_POINTS);

    glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, countQuery);

    //It's ok to use either fid because both raw and filtered have the same amount of vertices
    glDrawTransformFeedback(GL_POINTS, rawFeedback.fid);

    glEndTransformFeedback();

    glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);

    glGetQueryObjectuiv(countQuery, GL_QUERY_RESULT, &count);

    glDisable(GL_RASTERIZER_DISCARD);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    initProgram->Unbind();

    glFinish();
}

void DynamicModel::renderPointCloud(pangolin::OpenGlMatrix mvp,
                                   const float threshold,
                                   const bool drawUnstable,
                                   const bool drawNormals,
                                   const bool drawColors,
                                   const bool drawPoints,
                                   const bool drawWindow,
                                   const bool drawTimes,
                                   const int time,
                                   const int timeDelta)
{
    std::shared_ptr<Shader> program = drawPoints ? drawProgram : drawSurfelProgram;

    program->Bind();

    program->setUniform(Uniform("MVP", mvp));

    program->setUniform(Uniform("threshold", threshold));

    program->setUniform(Uniform("colorType", (drawNormals ? 1 : drawColors ? 2 : drawTimes ? 3 : 0)));

    program->setUniform(Uniform("unstable", drawUnstable));

    program->setUniform(Uniform("drawWindow", drawWindow));

    program->setUniform(Uniform("time", time));

    program->setUniform(Uniform("timeDelta", timeDelta));

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    //This is for the point shader
    program->setUniform(Uniform("pose", pose));

    glBindBuffer(GL_ARRAY_BUFFER, vbos[target].first);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, 0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 1));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 2));

    glDrawTransformFeedback(GL_POINTS, vbos[target].second);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    program->Unbind();
}

const std::pair<GLuint, GLuint> & DynamicModel::model()
{
    return vbos[target];
}

void DynamicModel::fuse(const Eigen::Matrix4f & pose,
                       const int & time,
                       GPUTexture * rgb, // current frame rgb
                       GPUTexture * depthRaw, // current frame raw depth
                       GPUTexture * depthFiltered, // current frame filter depth
                       GPUTexture * indexMap, //predict color
                       GPUTexture * vertConfMap, // predict vert confidence
                       GPUTexture * colorTimeMap, // predict color and time
                       GPUTexture * normRadMap, // predict normal
                       const float depthCutoff,
                       const float confThreshold,
                       const float weighting)
{
	/*
    TICK("Fuse::Data");
    //This first part does data association and computes the vertex to merge with, storing
    //in an array that sets which vertices to update by index
    //updateId = 1 is to update, updateId = 2 is unstable vertex
    frameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, renderBuffer.width, renderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    dataProgram->Bind();

    dataProgram->setUniform(Uniform("cSampler", 0));
    dataProgram->setUniform(Uniform("drSampler", 1));
    dataProgram->setUniform(Uniform("drfSampler", 2));
    dataProgram->setUniform(Uniform("indexSampler", 3));
    dataProgram->setUniform(Uniform("vertConfSampler", 4));
    dataProgram->setUniform(Uniform("colorTimeSampler", 5));
    dataProgram->setUniform(Uniform("normRadSampler", 6));
    dataProgram->setUniform(Uniform("time", (float)time));
    dataProgram->setUniform(Uniform("weighting", weighting)); // 1

    dataProgram->setUniform(Uniform("cam", Eigen::Vector4f(Intrinsics::getInstance().cx(),
                                                     Intrinsics::getInstance().cy(),
                                                     1.0 / Intrinsics::getInstance().fx(),
                                                     1.0 / Intrinsics::getInstance().fy())));
    dataProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    dataProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));
    dataProgram->setUniform(Uniform("scale", (float)IndexMap::FACTOR)); // 1
    dataProgram->setUniform(Uniform("texDim", (float)TEXTURE_DIMENSION)); // 3072
    dataProgram->setUniform(Uniform("pose", pose));
    dataProgram->setUniform(Uniform("maxDepth", depthCutoff));

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, uvo);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, newUnstableFid);

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, newUnstableVbo);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, rgb->texture->tid);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depthRaw->texture->tid);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, depthFiltered->texture->tid);

    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, indexMap->texture->tid);

    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, vertConfMap->texture->tid);

    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_2D, colorTimeMap->texture->tid);

    glActiveTexture(GL_TEXTURE6);
    glBindTexture(GL_TEXTURE_2D, normRadMap->texture->tid);

    glBeginTransformFeedback(GL_POINTS);

    glDrawArrays(GL_POINTS, 0, uvSize);

    glEndTransformFeedback();

    frameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    glDisableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    dataProgram->Unbind();

    glPopAttrib();

    glFinish();
    TOCK("Fuse::Data");
    */

    TICK("Fuse::Update");
    //Next we update the vertices at the indexes stored in the update textures
    //Using a transform feedback conditional on a texture sample
    //update vertex by new = (a*w_a + b*w_b) / (w_a + w_b)
    updateProgram->Bind();

    updateProgram->setUniform(Uniform("gSampler", 0));
    updateProgram->setUniform(Uniform("cSampler", 1));
    updateProgram->setUniform(Uniform("texDim", (float)TEXTURE_DIMENSION));
    updateProgram->setUniform(Uniform("time", time));
    updateProgram->setUniform(Uniform("cam", Eigen::Vector4f(Intrinsics::getInstance().cx(),
                                                     Intrinsics::getInstance().cy(),
                                                     1.0 / Intrinsics::getInstance().fx(),
                                                     1.0 / Intrinsics::getInstance().fy())));
    updateProgram->setUniform(Uniform("threshold", 0.0f));
    updateProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    updateProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));
    updateProgram->setUniform(Uniform("maxDepth", depthCutoff));

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, uvo);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glEnable(GL_RASTERIZER_DISCARD);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, vbos[renderSource].second);

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbos[renderSource].first);

    glBeginTransformFeedback(GL_POINTS);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthFiltered->texture->tid);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, rgb->texture->tid);

    glDrawArrays(GL_POINTS, 0, Resolution::getInstance().numPixels());

    // glDrawTransformFeedback(GL_POINTS, vbos[target].second);

	glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    glEndTransformFeedback();

    glDisable(GL_RASTERIZER_DISCARD);

    

    glDisableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    updateProgram->Unbind();

    std::swap(target, renderSource);

    glFinish();
    TOCK("Fuse::Update");
}

std::vector<float> DynamicModel::psdf(const std::vector<Eigen::Vector3f>& warped, DeviceArray2D<ushort>& cur_depth) {
    Projector projector(Intrinsics::getInstance().fx(),
                        Intrinsics::getInstance().fy(),
                        Intrinsics::getInstance().cx(),
                        Intrinsics::getInstance().cy());

    std::vector<float4, std::allocator<float4>> point_type(warped.size());
    for(int i = 0; i < warped.size(); i++)
    {
        point_type[i].x = warped[i][0];
        point_type[i].y = warped[i][1];
        point_type[i].z = warped[i][2];
        point_type[i].w = 0.f;
    }

    DeviceArray2D<float4> points;
    points.upload(point_type, cur_depth.cols());

    cuda_project_and_remove(cur_depth, points, projector);


}

void DynamicModel::clean(const Eigen::Matrix4f & pose,
                        const int & time,
                        GPUTexture * indexMap,
                        GPUTexture * vertConfMap,
                        GPUTexture * colorTimeMap,
                        GPUTexture * normRadMap,
                        GPUTexture * depthMap,
                        const float confThreshold,
                        std::vector<float> & graph,
                        const int timeDelta,
                        const float maxDepth,
                        const bool isFern)
{
    assert(graph.size() / 16 < MAX_NODES);

    if(graph.size() > 0)
    {
        //Can be optimised by only uploading new nodes with offset
        glBindTexture(GL_TEXTURE_2D, deformationNodes.texture->tid);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, graph.size(), 1, GL_LUMINANCE, GL_FLOAT, graph.data());
    }

    TICK("Fuse::Copy");
    //Next we copy the new unstable vertices from the newUnstableFid transform feedback into the global map
    unstableProgram->Bind();
    unstableProgram->setUniform(Uniform("time", time));
    unstableProgram->setUniform(Uniform("confThreshold", confThreshold));
    unstableProgram->setUniform(Uniform("scale", (float)IndexMap::FACTOR));
    unstableProgram->setUniform(Uniform("indexSampler", 0));
    unstableProgram->setUniform(Uniform("vertConfSampler", 1));
    unstableProgram->setUniform(Uniform("colorTimeSampler", 2));
    unstableProgram->setUniform(Uniform("normRadSampler", 3));
    unstableProgram->setUniform(Uniform("nodeSampler", 4));
    unstableProgram->setUniform(Uniform("depthSampler", 5));
    unstableProgram->setUniform(Uniform("nodes", (float)(graph.size() / 16)));
    unstableProgram->setUniform(Uniform("nodeCols", (float)NODE_TEXTURE_DIMENSION));
    unstableProgram->setUniform(Uniform("timeDelta", timeDelta));
    unstableProgram->setUniform(Uniform("maxDepth", maxDepth));
    unstableProgram->setUniform(Uniform("isFern", (int)isFern));

    Eigen::Matrix4f t_inv = pose.inverse();
    unstableProgram->setUniform(Uniform("t_inv", t_inv));

    unstableProgram->setUniform(Uniform("cam", Eigen::Vector4f(Intrinsics::getInstance().cx(),
                                                         Intrinsics::getInstance().cy(),
                                                         Intrinsics::getInstance().fx(),
                                                         Intrinsics::getInstance().fy())));
    unstableProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    unstableProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));

    glBindBuffer(GL_ARRAY_BUFFER, vbos[target].first);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, 0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f)));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 2));

    glEnable(GL_RASTERIZER_DISCARD);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, vbos[renderSource].second);

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbos[renderSource].first);

    glBeginTransformFeedback(GL_POINTS);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, indexMap->texture->tid);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, vertConfMap->texture->tid);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, colorTimeMap->texture->tid);

    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, normRadMap->texture->tid);

    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, deformationNodes.texture->tid);

    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_2D, depthMap->texture->tid);

    glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, countQuery);

    glDrawTransformFeedback(GL_POINTS, vbos[target].second);

    glBindBuffer(GL_ARRAY_BUFFER, newUnstableVbo);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, 0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f)));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 2));

    glDrawTransformFeedback(GL_POINTS, newUnstableFid);

    glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);

    glGetQueryObjectuiv(countQuery, GL_QUERY_RESULT, &count);

    glEndTransformFeedback();

    glDisable(GL_RASTERIZER_DISCARD);

    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    unstableProgram->Unbind();

    std::swap(target, renderSource);

    glFinish();
    TOCK("Fuse::Copy");
}

unsigned int DynamicModel::lastCount()
{
    return count;
}

Eigen::Vector4f * DynamicModel::downloadMap()
{
    glFinish();

    Eigen::Vector4f * vertices = new Eigen::Vector4f[count * 3];

    memset(&vertices[0], 0, count * Vertex::SIZE);

    GLuint downloadVbo;

    //generate downloadVbo
    glGenBuffers(1, &downloadVbo);
    glBindBuffer(GL_ARRAY_BUFFER, downloadVbo);
    glBufferData(GL_ARRAY_BUFFER, bufferSize, 0, GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    //bind read to first
    //bind write to download
    glBindBuffer(GL_COPY_READ_BUFFER, vbos[renderSource].first);
    glBindBuffer(GL_COPY_WRITE_BUFFER, downloadVbo);

    //copy from read to write
    //copy from write to vertices
    glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, count * Vertex::SIZE);
    glGetBufferSubData(GL_COPY_WRITE_BUFFER, 0, count * Vertex::SIZE, vertices);

    glBindBuffer(GL_COPY_READ_BUFFER, 0);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
    glDeleteBuffers(1, &downloadVbo);

    glFinish();

    return vertices;
}


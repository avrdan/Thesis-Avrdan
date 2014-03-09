#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// extension lib
#include <GL/glew.h>
// math lib
#include <glm/glm.hpp>

// glm extensions
#include <glm/gtc/matrix_transform.hpp>

// window lib
#include <GLFW/glfw3.h>

// OGL
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>


#include <vector>
#include <assimp/Importer.hpp> // C++ importer interface
#include <assimp/scene.h> // Output data structure
#include <assimp/postprocess.h> // Post processing flags

#include "common/loadShader.h"
#include "common/objloader.h"
#include "common/controls.h"
#include "common/textureLoader.h"


using namespace std;
using namespace glm;

// read our .obj file
vector<vec3> vertices;
vector<vec2> uvs;
vector<vec3> normals;

bool use_uvs = true;

int main()
{
    // initialize mesh data from obj file
    bool res = loadOBJ("teapot2.obj", vertices, uvs, normals);

    if(uvs.size() == 0)
    {
        use_uvs = false;
    }
    else
    {
        // invert v for dds
        for(vector<vec2>::iterator it = uvs.begin(); it != uvs.end(); ++it) {
           //cout << it->y << " ";
           it->y = 1 - it->y;
         }
     }

    if (!res)
    {
        return -1;
    }

    // init glfw
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x Antialiasing
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);


    // open a window and create its ogl context
    GLFWwindow* window;
    window = glfwCreateWindow(1024, 768, "Template", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "Failed to open GLFW window\n");
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    // init glew
    glewExperimental = true; // required in core p
    if (glewInit() != GLEW_OK)
    {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }

    // keyboard input
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);


    /*GLenum error = glGetError();
    const GLubyte* es = glewGetErrorString(error);
       cout << "ERROR: " << es << endl;
    */

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    GLuint programID = LoadShaders("TriangleVertexShader.vert",
                                   "TriangleFragmentShader.frag");

    cout << "LOADED SHADERS" << endl;
    /*
    // an array of 3 vectors; 3 vertices
    static const GLfloat g_vertex_buffer_data[] =
    {
      -1.0f, -1.0f, 0.0f,
       1.0f, -1.0f, 0.0f,
       0.0f,  1.0f, 0.0f

    // give our vertices to OGL
    //glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data),
      //           g_vertex_buffer_data, GL_STATIC_DRAW);

    };*/

    // vertex buffer
    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(vec3),
                 &vertices[0], GL_STATIC_DRAW);

    // normal buffer
    GLuint normalbuffer;
    glGenBuffers(1, &normalbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(vec3),
                 &normals[0], GL_STATIC_DRAW);

    // uv buffer
    GLuint uvbuffer;
    if(use_uvs)
    {

        glGenBuffers(1, &uvbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(vec2),
                     &uvs[0], GL_STATIC_DRAW);

    }

     glClearColor(0.0f, 0.0f, 0.4f, 1.0f);
     // Enable depth test
     glEnable(GL_DEPTH_TEST);
     // Accept fragment if it closer to the camera than the former one
     glDepthFunc(GL_LESS);

     // Cull triangles which normal is not towards the camera
     glEnable(GL_CULL_FACE);
    // load the shaders!!
    //GLuint programID = LoadShaders("/Users/avrdan/Work/DIKU/Thesis/Code/OpenGLTemplate/TriangleVertexShader.vert",
      //                             "/Users/avrdan/Work/DIKU/Thesis/Code/OpenGLTemplate/TriangleFragmentShader.frag");

    // SET UP MVP

    // projection matrix
    // 45 fov, 4:3 aspect; [0.1, 1000]

    mat4 ProjectionMatrix;
     /*mat4 Projection = perspective(45.0f, 4.0f/ 3.0f, 0.1f, 100.0f);
    // camera matrix
    mat4 View = lookAt(
                vec3(4, 3, 3), // eye vector
                vec3(0, 0, 0), // look at origin
                vec3(0, 1, 0)  // y vector up
                );
    */
    mat4 ViewMatrix;

     // model matrix (identity matrix; triangle in origin)
    mat4 ModelMatrix = mat4(1.0f); // changes for each model

    // the ModelViewProjectionMatrix
    mat4 MVP; //= Projection * View * Model; // inverse, due to matrix mult

    // get a handle for the MVP, M, V uniforms
    GLuint MatrixID      = glGetUniformLocation(programID, "MVP");
    GLuint ModelMatrixID = glGetUniformLocation(programID, "M");
    GLuint ViewMatrixID  = glGetUniformLocation(programID, "V");

    if (use_uvs)
    {
        // Load the texture
        //GLuint Texture = loadBMP_custom("uvtemplate.bmp");
        //GLuint Texture  = loadDDS("uvtemplate.dds");
        GLuint Texture = loadDDS("uvmap2.dds");

        // Get a handle for our "textureSampler"
        GLuint TextureID  = glGetUniformLocation(programID, "textureSampler");

        cout << "texture: " << Texture << endl;
    }

   glUseProgram(programID);
    // light
    GLuint LightID = glGetUniformLocation(programID, "LightPosition_worldspace");


    vec3 lightPos = vec3(4.0f, 4.0f, 4.0f);

    //cout << "START DRAWING..." << endl;


    do
    {
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(programID);

        // compute matrices;
        computeMatricesFromInputs(window);
        ProjectionMatrix = getProjectionMatrix();
        ViewMatrix = getViewMatrix();
        ModelMatrix = mat4(1.0f);
        MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

        // send out transformation to the currently bound shader
        // in the "MVP" uniform
        // for each model (in this case just once)
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        // send out other matrices
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

        // send out light uniform
        // light pos (fixed)
        glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);

        /*if(use_uvs)
        {
            // bind texture in Texture Unit 0
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, Texture);
            // bind uniform to TU 0
            glUniform1i(TextureID, 0);
            //glBindSampler(0, TextureID);
        }*/


        // draw stuff here

        // vertex array
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
            0,          // attrib 0
            3,          // size
            GL_FLOAT,   // type
            GL_FALSE,   // normalized?
            0,          // stride
            (void*)0    // array buffer offset
        );


        // normal array
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
        glVertexAttribPointer(
            1,          // attrib
            3,          // size
            GL_FLOAT,   // type
            GL_FALSE,   // normalized?
            0,          // stride
            (void*)0    // array buffer offset
        );

        if(use_uvs)
        {
            // uv array
            glEnableVertexAttribArray(2);
            glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
            glVertexAttribPointer(
                2,          // attrib
                2,          // size
                GL_FLOAT,   // type
                GL_FALSE,   // normalized?
                0,          // stride
                (void*)0    // array buffer offset
            );
        }


        // start from vertex 0
        // 3 number of vertices
        glDrawArrays(GL_TRIANGLES, 0, (vertices.size()));

        //cout << "DRAWING...DONE!" << endl;

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        if(use_uvs)
        {
            glDisableVertexAttribArray(2);
        }
        glfwSwapBuffers(window);
        glfwPollEvents();


        /*GLenum error = glGetError();

            if (error != GL_NO_ERROR)
            {
                printf("Error %s occured at %d", error, "Location");
            }*/
    }
    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0);

    // Cleanup VBO & Shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &normalbuffer);
    /*if(use_uvs)
    {   glDeleteBuffers(1, &uvbuffer);
        glDeleteTextures(1, &TextureID);
    }*/
    glDeleteProgram(programID);

    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}


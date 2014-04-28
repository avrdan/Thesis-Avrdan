#include "frameBuffersScene.h"
#include <iostream>

FrameBuffersScene::FrameBuffersScene()
{
	vertexShaderSource = "./assets/shaders/pre_process.vert";
	fragmentShaderSource = "./assets/shaders/pre_process.frag";
	vertexShaderPostProcessSource = "./assets/shaders/post_process.vert";
	fragmentShaderPostProcessSource = "./assets/shaders/post_process.frag";

	texSource = "./assets/textures/matrix1k.jpg";
}


FrameBuffersScene::~FrameBuffersScene()
{
}

void FrameBuffersScene::initScene(GLFWwindow *window)
{
	glfwGetWindowSize(window, &width, &height);

	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);

	CreateStaticSceneObjects3(&uiVAOSceneObjects, vboSceneObjects);

	// create and bind framebuffer
	glGenFramebuffers(1, &fb);
	glBindFramebuffer(GL_FRAMEBUFFER, fb);

	glEnable(GL_TEXTURE_2D);
	bool t = texture.loadTexture2D(texSource, true);
	if (t)
	{
		cout << "TEXTURE LOADED SUCCESSFULLY" << endl;
	}
	texture.setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_BILINEAR_MIPMAP);
	
	// generate full viewport texture
	glGenTextures(1, &fb_tex);
	glBindTexture(GL_TEXTURE_2D, fb_tex);
	// IMPORTANT: texture must be COMPLETE (mipmaps must be specified..
	// or the following parameters can be used if there are none)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

	glTexImage2D(
		GL_TEXTURE_2D,
		0,
		GL_RGBA,
		width,
		height,
		0,
		GL_RGBA,
		GL_UNSIGNED_BYTE,
		NULL
		);
	/*
	// Poor filtering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);*/

	// generate render buffer
	glGenRenderbuffers(1, &rb);
	glBindRenderbuffer(GL_RENDERBUFFER, rb);
	glRenderbufferStorage(
		GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
	// add render buffer to frame buffer
	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rb
		);

	// bind texture to framebuffer
	glFramebufferTexture(
		GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, fb_tex, 0);


	// list of color buffers
	GLenum draw_bufs[] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, draw_bufs);

	// bind default frame buffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	programID			 = LoadShaders(vertexShaderSource, fragmentShaderSource);
	programPostProcessID = LoadShaders(vertexShaderPostProcessSource, fragmentShaderPostProcessSource);

	glUseProgram(programID);

	// Always check that our framebuffer is ok
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		cout << "FRAME BUFFER NOT COMPLETE!" << endl;
	}

}

void FrameBuffersScene::renderScene(GLFWwindow *window)
{
	// bind the second (render-to-texture) framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, fb);
	glViewport(0, 0, width, height);
	// clear the framebuffer's colour and depth buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render scene as normal here
	int iSamplerLoc = glGetUniformLocation(programID, "tex");
	glUniform1i(iSamplerLoc, 0);

	// bind the quad's VAO
	glBindVertexArray(uiVAOSceneObjects);
	// bind texture to write into second fb
	texture.bindTexture();
	// draw the quad
	glDrawArrays(GL_TRIANGLES, 0, 6);
	// works until here...
	
	// bind default framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, width, height);
	// clear the framebuffer's colour and depth buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// our post-processing shader for the screen-space quad
	glUseProgram(programPostProcessID);
	iSamplerLoc = glGetUniformLocation(programPostProcessID, "tex");
	glUniform1i(iSamplerLoc, 0);
	// bind the quad's VAO
	glBindVertexArray(uiVAOSceneObjects);
	// activate the first texture slot and put texture from previous pass in it
	//texture.bindTexture();
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, fb_tex);

	// draw the quad
	glDrawArrays(GL_TRIANGLES, 0, 6);
	
	// flip drawn framebuffer onto the display
	glfwSwapBuffers(window);
	glfwPollEvents();

	glUseProgram(programID);
}

void FrameBuffersScene::releaseScene()
{
	glDeleteProgram(programID);
	glDeleteProgram(programPostProcessID);

	vboSceneObjects.releaseVBO();
	glDeleteVertexArrays(1, &uiVAOSceneObjects);

	texture.releaseTexture();

	glDeleteFramebuffers(1, &fb);
	glDeleteTextures(1, &fb_tex);
	glDeleteRenderbuffers(1, &rb);
}
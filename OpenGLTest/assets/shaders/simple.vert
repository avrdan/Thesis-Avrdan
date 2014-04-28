#version 330

// input vertex data
layout(location = 0) in vec3 vp;

uniform mat4 P;
uniform mat4 MV;
uniform mat4 invP;
uniform mat4 invMV;
void main()
{
	/*The transformed clip space position c of a
	world space vertex v is obtained by transforming 
	v with the product of the projection matrix P 
	and the modelview matrix MV
 
	c = P MV v
 
	So, if we could solve for v, then we could 
	genrerate vertex positions by plugging in clip 
	space positions. For your frustum, one line 
	would be between the clip space positions 
 
	(-1,-1,near) and (-1,-1,far), 
 
	the lower left edge of the frustum, for example.
 
	NB: If you would like to mix normalized device 
	coords (x,y) and eye space coords (near,far), 
	you need an additional step here. Modify your 
	clip position as follows
 
	c' = (c.x * c.z, c.y * c.z, c.z, c.z)
 
	otherwise you would need to supply both the z 
	and w for c, which might be inconvenient. Simply 
	use c' instead of c below.
 
 
	To solve for v, multiply both sides of the equation above with 
 
		  -1       
	(P MV) 
 
	This gives
 
		  -1      
	(P MV)   c = v
 
	This is equivalent to
 
	  -1  -1      
	MV   P   c = v
 
	 -1
	P   is given by
 
	|(r-l)/(2n)     0         0      (r+l)/(2n) |
	|     0    (t-b)/(2n)     0      (t+b)/(2n) |
	|     0         0         0         -1      |
	|     0         0   -(f-n)/(2fn) (f+n)/(2fn)|
 
	where l, r, t, b, n, and f are the parameters in the glFrustum() call.
 
	If you don't want to fool with inverting the 
	model matrix, the info you already have can be 
	used instead: the forward, right, and up 
	vectors, in addition to the eye position.
 
	First, go from clip space to eye space
 
		 -1   
	e = P   c
 
	Next go from eye space to world space
 
	v = eyePos - forward*e.z + right*e.x + up*e.y
 
	assuming x = right, y = up, and -z = forward.
	*/
	vec4 fVp = invMV * invP * vec4(vp, 1.0);
	gl_Position = P * MV * fVp;
}
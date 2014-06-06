#version 330

//in vec4 theColor;
smooth in vec4 pos;
out vec4 outputColor;

vec2 resolution;

vec3 heat (float v) {
    float value = 1.-v;
    return (0.5+0.5*smoothstep(0.0, 0.1, value))*vec3(
      smoothstep(0.5, 0.3, value),
      value < 0.3 ? smoothstep(0.0, 0.3, value) : smoothstep(1.0, 0.6, value),
      smoothstep(0.4, 0.6, value)
    );
  }

/*float sum = 0.0;

void main () {
	resolution.x = 1920;
	resolution.y = 1080;   
    vec2 p = gl_FragCoord.xy / resolution.xy;
    */
    /*for (int i=0; i<resolution.x * resolution.y; ++i) { //if (i == pointsLength) break;
      //vec4 point = points[i];
      //sum += point.w*smoothstep(point.z, 0.0, distance(point.xy, p));
	  sum += gl_FragCoord.w*smoothstep(gl_FragCoord.z, 0.0, distance(gl_FragCoord.xy, p));
    }*/
	/*sum += gl_FragCoord.w*smoothstep(gl_FragCoord.z, 0.0, distance(gl_FragCoord.xy, p));
    gl_FragColor = vec4(heat(sum), 1.0);
  }
  */

void main()
{
	//outputColor = vec4(theColor, 1.0);
	float near = 2;
	// what is the best far plane??
	// the original perspective plane is much too large(1000)
	// to show anything meaningful, unless the object(s) in question 
	// fill the whole scene
	float far = 10;
	
	vec4 colors[3];
	colors[0] = vec4(0, 0, 1, 1); // blue
	colors[1] = vec4(1, 1, 0, 1); // yellow
	colors[2] = vec4(1, 0, 0, 1); // red

	float depth = gl_FragCoord.z / gl_FragCoord.w;
	//float depth = gl_FragCoord.z / far;

	int ix = (depth > far/2) ? 0 : 1;
	
	//vec4 heat = mix(colors[ix+1], colors[ix], (depth - float(ix) * 0.5)/0.5);
	//vec4 heat = mix(colors[ix+1], colors[ix], smoothstep(near, far, (depth - float(ix) * 0.5)/0.5));
	//vec4 heat = mix(colors[ix+1], colors[ix], smoothstep(near, far, gl_FragCoord.z / gl_FragCoord.w));
	//vec4 heat = mix(colors[2], colors[0], smoothstep(near, far, gl_FragCoord.z / gl_FragCoord.w));
    outputColor = vec4(heat(depth), 1.0);
	//outputColor = vec4(gl_FragCoord.x, gl_FragCoord.y, gl_FragCoord.z, gl_FragCoord.w);
	//outputColor = vec4(depth, depth, depth, 1);
	//outputColor = mix(colors[0], colors[1], 0.1);
}
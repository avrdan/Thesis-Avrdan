#version 330 core

// interpolated values from the vertex shaders
in vec3 Position_worldspace;
in vec3 Normal_cameraspace;
in vec3 EyeDirection_cameraspace;
in vec3 LightDirection_cameraspace;
in vec2 UV;

// values that stay constant for the whole mesh
uniform vec3 LightPosition_worldspace;
uniform sampler2D textureSampler;

// output color
out vec3 color;

void main(void)
{   
    vec3 lightColor  = vec3(1, 1, 1);
    float lightPower = 50.0f;
    // no texture
    //vec3 MaterialDiffuseColor = vec3(0, 1, 0);
    // with texture
   
    vec3 MaterialDiffuseColor = texture(textureSampler, UV).rgb;
    //vec3 MaterialDiffuseColor = vec3(UV.x, UV.y, 0);
    vec3 MaterialAmbientColor = vec3(0.1, 0.1, 0.1) * MaterialDiffuseColor;
    vec3 MaterialSpecularColor = vec3(1, 1, 1);
    
    // distance to the light
    float distance = length(LightPosition_worldspace - Position_worldspace);    
    
    // normal of the computed fragment
    vec3 n = normalize(Normal_cameraspace);
    // light direction
    vec3 l = normalize(LightDirection_cameraspace);
    
    // Cosine of the angle between the normal and the light direction,
    // clamped above 0
    //  - light is at the vertical of the triangle -> 1
    //  - light is perpendicular to the triangle -> 0
    //  - light is behind the triangle -> 0
    float cosTheta = clamp( dot( n,l ), 0,1 );
    
    // eye vector (towards the camera)
    vec3 E = normalize(EyeDirection_cameraspace);
    // direction in which the triangle reflects the light
    vec3 R = reflect(-l, n);
    
    // Cosine of the angle between the Eye vector and the Reflect vector,
    // clamped above 0
    //  - looking into the reflection -> 1
    //  - looking elsewhere -> < 1
    float cosAlpha = clamp( dot(E, R), 0, 1);
    
   color = MaterialAmbientColor + 
           MaterialDiffuseColor * lightColor * lightPower * cosTheta / (distance*distance) +
           MaterialSpecularColor * lightColor * lightPower * pow(cosAlpha, 5) / (distance*distance) ;
}

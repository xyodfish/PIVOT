#include "kinematic_viewer/kinematic_shader_utils.h"

#include <iostream>

namespace kinematic_viewer::detail {

    const char* kMeshVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;
void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    TexCoords = aTexCoords;
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

    const char* kMeshFragmentShader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;
uniform vec3 viewPos;
uniform vec3 diffuseColor;
uniform vec3 sunDirection;
uniform vec3 lightColor;
uniform vec3 skyColor;
uniform vec3 groundColor;
uniform float diffuseScale;
uniform float specularScale;
uniform int upAxis;
uniform bool hasTexture;
uniform float materialAlpha;
uniform sampler2D texture_diffuse1;
out vec4 color;

const float PI = 3.14159265359;

vec3 toLinear(vec3 c) {
    return pow(max(c, vec3(0.0)), vec3(2.2));
}

vec3 toSrgb(vec3 c) {
    return pow(max(c, vec3(0.0)), vec3(1.0 / 2.2));
}

void main() {
    vec3 albedo = toLinear(diffuseColor);
    if (hasTexture) {
        albedo *= toLinear(texture(texture_diffuse1, TexCoords).rgb);
    }

    vec3 N = normalize(Normal);
    vec3 V = normalize(viewPos - FragPos);
    vec3 L = normalize(sunDirection);
    vec3 H = normalize(V + L);

    float NdotL = max(dot(N, L), 0.0);
    float NdotH = max(dot(N, H), 0.0);

    float shininess = 64.0;
    float normFactor = (shininess + 2.0) / (8.0 * PI);
    vec3 F0 = vec3(0.04);
    vec3 diffuse = albedo * lightColor * NdotL * 2.5 * diffuseScale;
    vec3 spec = F0 * lightColor * normFactor * pow(NdotH, shininess) * NdotL * specularScale;

    vec3 up = vec3(0.0, 0.0, 1.0);
    if (upAxis == 0) {
        up = vec3(1.0, 0.0, 0.0);
    } else if (upAxis == 1) {
        up = vec3(0.0, 1.0, 0.0);
    }
    float sky_fac = dot(N, up) * 0.5 + 0.5;
    vec3 ambient = mix(toLinear(groundColor), toLinear(skyColor), sky_fac) * albedo;

    vec3 lit = ambient + diffuse + spec;
    color = vec4(toSrgb(lit), materialAlpha);
}
)";

    const char* kLineVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
uniform mat4 view;
uniform mat4 projection;
out vec3 Color;
void main() {
    Color = aColor;
    gl_Position = projection * view * vec4(aPos, 1.0);
}
)";

    const char* kLineFragmentShader = R"(
#version 330 core
in vec3 Color;
out vec4 FragColor;
void main() {
    FragColor = vec4(Color, 1.0);
}
)";

    const char* kPointVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float pointSize;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    gl_PointSize = pointSize;
}
)";

    const char* kPointFragmentShader = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(vColor, 0.88);
}
)";

    GLuint compileShader(GLenum type, const char* src) {
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);
        GLint ok = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (ok != GL_TRUE) {
            char log[1024];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            std::cerr << "Shader compile failed: " << log << std::endl;
        }
        return shader;
    }

    GLuint createProgram(const char* vsSrc, const char* fsSrc) {
        GLuint vs      = compileShader(GL_VERTEX_SHADER, vsSrc);
        GLuint fs      = compileShader(GL_FRAGMENT_SHADER, fsSrc);
        GLuint program = glCreateProgram();
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);
        GLint ok = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &ok);
        if (ok != GL_TRUE) {
            char log[1024];
            glGetProgramInfoLog(program, sizeof(log), nullptr, log);
            std::cerr << "Program link failed: " << log << std::endl;
        }
        glDeleteShader(vs);
        glDeleteShader(fs);
        return program;
    }

}  // namespace kinematic_viewer::detail

namespace kinematic_viewer {

    GLuint createKinematicMeshProgram() {
        return detail::createProgram(detail::kMeshVertexShader, detail::kMeshFragmentShader);
    }

    GLuint createKinematicLineProgram() {
        return detail::createProgram(detail::kLineVertexShader, detail::kLineFragmentShader);
    }

    GLuint createKinematicPointProgram() {
        return detail::createProgram(detail::kPointVertexShader, detail::kPointFragmentShader);
    }

}  // namespace kinematic_viewer

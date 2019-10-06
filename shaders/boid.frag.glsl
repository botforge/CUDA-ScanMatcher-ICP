#version 330

in vec4 vFragColor;
out vec4 fragColor;

void main() {
    fragColor.r = abs(vFragColor.r);
    fragColor.g = abs(vFragColor.g);
    fragColor.b = abs(vFragColor.b);
}

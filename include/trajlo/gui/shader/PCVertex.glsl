#version 330 core
layout (location = 0) in vec4 aPos;
out vec3 ourColor;
uniform mat4 pvm;

vec3 intensityToRainbow(float intensity) {
    float normalizedIntensity = intensity / 255.0;

    float r = abs(normalizedIntensity * 6.0 - 3.0) - 1.0;
    float g = 2.0 - abs(normalizedIntensity * 6.0 - 2.0);
    float b = 2.0 - abs(normalizedIntensity * 6.0 - 4.0);

    return clamp(vec3(r, g, b), 0.0, 1.0);
}

//vec3 intensityToRainbow(float intensity) {
//    float mappedIntensity = (intensity / 255.0) * 20.0 - 10.0;
//
//    float normalizedIntensity = (mappedIntensity + 10.0) / 20.0;
//
//    normalizedIntensity = 1.0 - normalizedIntensity;
//
//    float r = abs(normalizedIntensity * 6.0 - 3.0) - 1.0;
//    float g = 2.0 - abs(normalizedIntensity * 6.0 - 2.0);
//    float b = 2.0 - abs(normalizedIntensity * 6.0 - 4.0);
//
//    return clamp(vec3(r, g, b), 0.0, 1.0);
//}


void main() {
    gl_Position = pvm * vec4(aPos.xyz, 1.0f);

    int intensity=int(aPos.w);

    ourColor = intensityToRainbow(intensity);

//    int r,g,b;
//    if (intensity < 30)
//    {
//        r = 0;
//        g = int(intensity * 255 / 30) & 0xff;
//        b = 0xff;
//    }
//    else if (intensity < 90)
//    {
//        r = 0;
//        g = 0xff;
//        b = int((90 - intensity) * 255 / 60) & 0xff;
//    }
//    else if (intensity< 150)
//    {
//        r = ((intensity - 90) * 255 / 60) & 0xff;
//        g = 0xff;
//        b = 0;
//    }
//    else
//    {
//        r = 0xff;
//        g = int((255 - intensity) * 255 / (256 - 150)) & 0xff;
//        b = 0;
//    }
//
//    ourColor=vec3(r*1.0f/255,g*1.0f/255,b*1.0f/255);
}

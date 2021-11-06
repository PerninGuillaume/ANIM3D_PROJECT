#pragma once

/// Holds all state information relevant to a character as loaded using FreeType
struct Character {
    unsigned int TextureID; // ID handle of the glyph texture
    vcl::vec2   Size;      // Size of glyph
    vcl::vec2   Bearing;   // Offset from baseline to left/top of glyph
    unsigned int Advance;   // Horizontal offset to advance to next glyph
};

class textRender {
public:
    void setup_font(std::map<std::string,GLuint>& shaders);
    void renderText(std::string text, float x, float y, float scale, vcl::vec3 color);

private:
    GLuint textShaderId;
    std::map<GLchar, Character> characters;
    unsigned int VAO, VBO;
};

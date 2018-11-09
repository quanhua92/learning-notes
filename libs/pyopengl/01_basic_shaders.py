import glfw
import OpenGL.GL.shaders
import numpy as np
from OpenGL.GL import *


def main():
    # initalize glfw
    if not glfw.init():
        return

    window = glfw.create_window(800, 600, "My OpenGL Window", None, None)

    if not window:
        glfw.terminate()
        return

    glfw.make_context_current(window)

    # Create shaders
    vertex_shader = """
    #version 330
    in vec4 position;
    void main()
    {
        gl_Position = position;
    }
    """
    fragment_shader = """
    #version 330
    void main()
    {
        gl_FragColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
    }
    """
    shader = OpenGL.GL.shaders.compileProgram(
        OpenGL.GL.shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
        OpenGL.GL.shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER)
    )
    # Create triangle
    triangle = np.array([-0.5, -0.5, 0.0,
                0.5, -0.5, 0.0,
                0.0, 0.5, 0.0], dtype=np.float32)

    # Create buffer on GPU
    VBO = glGenBuffers(1)
    glBindBuffer(GL_ARRAY_BUFFER, VBO)

    glBufferData(GL_ARRAY_BUFFER, 9 * 4, triangle, GL_STATIC_DRAW) # 9 values * 4 bytes

    position = glGetAttribLocation(shader, "position")
    glVertexAttribPointer(position, 3, GL_FLOAT, GL_FALSE, 0, None)
    glEnableVertexAttribArray(position)

    glUseProgram(shader)

    glClearColor(0.2, 0.3, 0.2, 1.0)

    while not glfw.window_should_close(window):
        glfw.poll_events()

        glClear(GL_COLOR_BUFFER_BIT)

        glDrawArrays(GL_TRIANGLES, 0, 3) # 3 vertices

        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()

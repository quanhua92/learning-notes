import glfw
import OpenGL.GL.shaders
import numpy as np
import pyrr
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
    in vec3 position;
    in vec3 color;
    uniform mat4 transform;

    out vec3 newColor;

    void main()
    {
        gl_Position = transform * vec4(position, 1.0f);
        newColor = color;
    }
    """

    fragment_shader = """
    #version 330
    in vec3 newColor;
    out vec4 outColor;

    void main()
    {
        outColor = vec4(newColor, 1.0f);
    }
    """
    shader = OpenGL.GL.shaders.compileProgram(
        OpenGL.GL.shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
        OpenGL.GL.shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER)
    )
    # Create triangle
    #    POSITIONS      COLORS
    cube = np.array([
        -0.5, -0.5, 0.5, 1.0, 0.0, 0.0,
        0.5 , -0.5, 0.5, 0.0, 1.0, 0.0,
        0.5 ,  0.5, 0.5, 0.0, 0.0, 1.0,
        -0.5, 0.5, 0.5, 1.0, 1.0, 1.0,

        -0.5, -0.5, -0.5, 1.0, 0.0, 0.0,
        0.5 , -0.5, -0.5, 0.0, 1.0, 0.0,
        0.5 ,  0.5, -0.5, 0.0, 0.0, 1.0,
        -0.5, 0.5, -0.5, 1.0, 1.0, 1.0,
        ], dtype=np.float32)

    indices = np.array([
        0, 1, 2, 2, 3, 0,
        4, 5, 6, 6, 7, 4,
        4, 5, 1, 1, 0, 4,
        6, 7, 3, 3, 2, 6,
        5, 6, 2, 2, 1, 5,
        7, 4, 0, 0, 3, 7
    ], dtype=np.uint32)

    # Create buffer on GPU
    VBO = glGenBuffers(1)
    glBindBuffer(GL_ARRAY_BUFFER, VBO)
    glBufferData(GL_ARRAY_BUFFER, 8 * 6 * 4, cube, GL_STATIC_DRAW) # 6 * 4 values * 4 bytes

    EBO = glGenBuffers(1)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * 6 * 4, indices, GL_STATIC_DRAW) # 6 values * 4 bytes

    position = glGetAttribLocation(shader, "position")
    glVertexAttribPointer(position, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(0))
    glEnableVertexAttribArray(position)

    color = glGetAttribLocation(shader, "color")
    glVertexAttribPointer(color, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(12))
    # 12 is the starting position of color in the array. Since we have three 4 bytes for position, color will start at 12

    glEnableVertexAttribArray(color)

    glUseProgram(shader)

    glClearColor(0.2, 0.3, 0.2, 1.0)
    glEnable(GL_DEPTH_TEST)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)

    while not glfw.window_should_close(window):
        glfw.poll_events()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        rot_x = pyrr.Matrix44.from_x_rotation(0.5 * np.pi * glfw.get_time())
        rot_y = pyrr.Matrix44.from_y_rotation(0.08 * np.pi * glfw.get_time())

        transformLoc = glGetUniformLocation(shader, "transform")

        glUniformMatrix4fv(transformLoc, 1, GL_FALSE, rot_x * rot_y)

        glDrawElements(GL_TRIANGLES, 6 * 6, GL_UNSIGNED_INT, None)

        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()

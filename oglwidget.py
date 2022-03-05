from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt5.QtOpenGL import *

cubeVertices = [(1, 0.5, 1), (1, 0.5, -1), (1, -0.5, -1), (1, -0.5, 1),
                (-1, 0.5, 1), (-1, -0.5, -1), (-1, -0.5, 1), (-1, 0.5, -1)]
cubeQuads = [(0, 3, 6, 4), (2, 5, 6, 3), (1, 2, 5, 7), (1, 0, 4, 7), (7, 4, 6, 5), (2, 3, 0, 1)]
colors = [(0, 0, 1), (0, 1, 0), (0, 0, 1), (0, 1, 0), (1, 0, 0), (1, 0, 0)]


class OglWidget(QGLWidget):

    def __init__(self, parent=None):
        self.minX = 400
        self.minY = 400
        QGLWidget.__init__(self, parent)
        self.setMinimumSize(self.minX, self.minY)
        self.flag = True
        self.pos = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.quaternion = [0, 0, 0, 0]
        self.state = "quaternion"

    def initializeGL(self):
        glViewport(0, 0, self.minX, self.minY)
        gluPerspective(60, (self.minX / self.minY), 1, 50.0)
        glEnable(GL_DEPTH_TEST)
        glTranslatef(0.0, 0.0, -5)
        glClearColor(1, 1, 1, 1)
        glLineWidth(5)

    def cube(self):
        glBegin(GL_QUADS)
        for i, cubeQuad in enumerate(cubeQuads):
            for j, cubeVertex in enumerate(cubeQuad):
                glColor3fv(tuple([c / (j + 1) for c in colors[i]]))
                glVertex3fv(cubeVertices[cubeVertex])
        glEnd()

    def axis(self):
        glBegin(GL_LINES)
        glColor3fv((0, 0, 0))
        # glVertex3fv((3, -2.3, -2.3))
        # glVertex3fv((-3.3, -2.3, -2.3))
        # glVertex3fv((-2.3, -3.3, -2.3))
        # glVertex3fv((-2.3, 3, -2.3))
        # glVertex3fv((-2.3, -2.3, -3.3))
        # glVertex3fv((-2.3, -2.3, 3))
        glVertex3fv((6, 0, 0))
        glVertex3fv((-2, 0, 0))
        glVertex3fv((0, 6, 0))
        glVertex3fv((0, -1.5, 0))
        glVertex3fv((1, 1.5, 0))
        glVertex3fv((-6, -9, 0))
        glEnd()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()

        # update with angles
        if self.state == "euler":
            glRotatef(self.yaw, 0, 0, 1)
            glRotatef(self.pitch, 0, 1, 0)
            glRotatef(self.roll, 1, 0, 0)

        # update with Quaternion
        if self.state == "quaternion":
            q = self.quaternion
            # column major order
            glMultMatrixf([q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2, 2 * (q[1] * q[2] + q[0] * q[3]),
                          2 * (q[1] * q[3] - q[0] * q[2]), 0,
                          2 * (q[1] * q[2] - q[0] * q[3]), q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2,
                          2 * (q[2] * q[3] + q[0] * q[1]), 0,
                          2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]),
                          q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2, 0,
                          0, 0, 0, 1])

        self.cube()
        glPopMatrix()
        self.axis()

    def updateWithAngle(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.state = "euler"
        self.update()

    def updateWithQuaternion(self, Q):
        self.state = "quaternion"
        self.quaternion = Q
        self.update()

    def setPos(self, pos):
        self.pos = pos

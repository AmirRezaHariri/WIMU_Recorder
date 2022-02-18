from PyQt5.QtWidgets import*
from matplotlib.backends.backend_qt5agg import FigureCanvas
from matplotlib.figure import Figure


class MplWidget(QWidget):
    
    def __init__(self, parent = None):
        QWidget.__init__(self, parent)
        figure = Figure(linewidth=1)
        # figure.set_facecolor((0.2, 0.2, 0.2))
        # figure.set_edgecolor((0.2, 0.2, 0.2))
        self.canvas = FigureCanvas(figure)
        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.canvas)
        self.canvas.axes = self.canvas.figure.add_subplot(111)
        # self.canvas.axes.set_facecolor((0.3, 0.3, 0.3))
        self.canvas.axes.grid("True")
        self.setLayout(vertical_layout)
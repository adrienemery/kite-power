import SimpleCV
import numpy as np


class KiteTracker(object):

    def __init__(self, path='static_kite.avi', hide_window=False, show_tail=False):

        if path == 'cam':
            self.cam = SimpleCV.Camera(threaded=True)

        elif path == 'none':
            pass

        else:
            self.vid = SimpleCV.VirtualCamera(path, 'video')

        self.display = SimpleCV.Display()
        self.value = 200
        self.tail = []
        self.filter = 0
        self.show_tail = show_tail
        self.hide_window = hide_window
        self.pos = None
        self.path = path
        self.targets = {1: (150, 200), 2: (600, 400), 3: (600, 200), 4: (150, 400)}
        self.current_target = 1
        self.target_radius = 100

        # create log file
        with open('log.txt', 'w') as f:
            f.write('x_pos, y_pos\n')

    def get_pos(self):
        """
        Returns x,y position of the kite as a tuple (x,y)
        """
        return self.pos

    def set_filter(self, number):
        """
        Select which filter to use to track the kite.
        0 - default black color distance
        1 - Red color distance
        2 -
        3 -
        """
        self.filter = number

    def get_current_target(self):
        return self.targets[self.current_target]

    def update(self):
        """
        Runs color tracking algorithms on web cam stream or video from file.
        Call this once a loop.
        """

        if self.path is 'cam':
            img = self.cam.getImage()
        else:
            img = self.vid.getImage()
            img = img.resize(720, 480)

        if self.filter == 0:
            dist_img = img.colorDistance(color=SimpleCV.Color.BLACK)
            dist_img = img - dist_img
            #dist_img = dist_img.colorDistance(color=SimpleCV.Color.RED)
            #dist_img = dist_img.threshold(value).erode(3).invert()

        elif self.filter == 1:
            dist_img = img.colorDistance(color=SimpleCV.Color.WHITE)
            #dist_img = img - dist_img
            filtered_img = dist_img.threshold(170).dilate()
            dist_img = filtered_img

        blobs = dist_img.findBlobs(minsize=200)
        if blobs:
            for blob in blobs:
                c = blob.centroid()
                self.pos = c
                img.dl().circle(c, 6, SimpleCV.Color.GREEN)
                self.tail.append(c)
                if len(self.tail) > 80:
                    self.tail = self.tail[1:]

                # write to file
                with open('log.txt', 'a') as f:
                    f.write(str(c[0]) + ',' + str(c[1]) + '\n')

        if self.tail and self.show_tail:
            for point in self.tail[:-2]:
                img.dl().circle(point, 6, SimpleCV.Color.RED)

        if self.pos:
            img.dl().line(self.pos, self.targets[self.current_target])

            v1 = np.mat(self.pos)
            v2 = np.mat(self.targets[self.current_target])
            dist = np.linalg.norm(v1 - v2)

            if dist < self.target_radius:
                self.current_target += 1
                if self.current_target > 4:
                    self.current_target = 1

            if len(self.tail) > 2:
                last_pos = self.tail[-2]
                pos = self.tail[-1]

                if not last_pos == pos:
                    # drawing heading based on last two points
                    v1 = np.mat(last_pos)
                    v2 = np.mat((pos))
                    v3 = v2 - v1
                    v3 = 50*v3/np.linalg.norm(v3) + v2

                    p1 = v1[0,0], v1[0,1]
                    p2 = v2[0,0], v2[0,1]
                    p3 = v3[0,0], v3[0,1]

                    img.dl().line(p2, p3, SimpleCV.Color.RED)

        if not self.hide_window:
            for key in self.targets:
                        img.dl().circle(self.targets[key], self.target_radius, SimpleCV.Color.BLUE)
            img.show()


if __name__ == '__main__':
    tracker = KiteTracker(path='foil_kite1_trim.avi')
    #tracker = KiteTracker(path='foil_kite1_tri')
    tracker.set_filter(1)

    while True:
        tracker.update()
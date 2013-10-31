import SimpleCV


class KiteTracker(object):

    def __init__(self, path='static_kite.avi', hide_window=False, show_tail=False):

        if path == 'cam':
            self.cam = SimpleCV.Camera()

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

        dist_img = img.colorDistance(color=SimpleCV.Color.BLACK)
        dist_img = img - dist_img
        #dist_img = dist_img.colorDistance(color=SimpleCV.Color.RED)
        #dist_img = dist_img.threshold(value).erode(3).invert()

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

        #dist_img.show()
        #filtered_img.show()

        if self.pos:
            img.dl().line(self.pos, (200, 200))
        if not self.hide_window:
            img.show()

    # TODO add color filter functions to get called based on user selection
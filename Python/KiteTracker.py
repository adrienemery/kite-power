import SimpleCV

vid = SimpleCV.VirtualCamera('static_kite.avi', 'video')

display = SimpleCV.Display()
value = 200
tail = []

# create log file
with open('log.txt', 'w') as f:
    f.write('x_pos, y_pos\n')

while display.isNotDone():
    img = vid.getImage()
    img = img.resize(720, 480)

    dist_img = img.colorDistance(color=SimpleCV.Color.BLACK)
    dist_img = img - dist_img
    #dist_img = dist_img.colorDistance(color=SimpleCV.Color.RED)
    #dist_img = dist_img.threshold(value).erode(3).invert()

    blobs = dist_img.findBlobs(minsize=200)
    if blobs:
        for blob in blobs:
            c = blob.centroid()
            img.dl().circle(c, 6, SimpleCV.Color.GREEN)
            tail.append(c)
            if len(tail) > 80:
                tail = tail[1:]

            # write to file
            with open('log.txt', 'a') as f:
                f.write(str(c[0]) + ',' + str(c[1]) + '\n')

    if tail:
        for point in tail[:-2]:
            img.dl().circle(point, 6, SimpleCV.Color.RED)

    #dist_img.show()
    #filtered_img.show()
    img.show()
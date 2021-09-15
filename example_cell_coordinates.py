import cv2
import numpy as np
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon


# Draw boxes.
# Correct with your desired polygons. This is an example of use for a chessboard case.
def draw_box(dst):
    all_boxes = []
	cells_unit = []
    corners = [[0,0],[0,0],[0,0],[0,0]]

	rows = ...	# Image size Y
	cols = ...	# Image size X
	boxes_rows = ... # Number of boxes in the Y direction
	boxes_cols = ... # Number of boxes in the X direction

	Hjump = ... # Size of box in Y direction
	Wjump = ... # Size of box in X direction

	px0 = --- # DO from initial x coordinate
	py0 = --- # DO from initial y coordinate

    for r in range(boxes_rows):
        W = Wjump

        px = px0
        for c in range(boxes_columns):
            H = Hjump

            corners[0] = [px , py]
            corners[1] = [px , py + H ]
            corners[2] = [px + W, py + H ]
            corners[3] = [px + W, py ]

            px += W

			# Correct maximum position of corners with image size
            for c in corners:
                if c[0] < 0:
                    c[0] = 0
                elif c[0] > cols:
                    c[0] = cols

                if c[1] < 0:
                    c[1] = 0
                elif c[1] > rows:
                    c[1] = rows


			# Paint the box in the image
            pts = np.array([corners[0],corners[1],corners[2],corners[3]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(dst,[pts],True,(0,255,0))

			# Create the box
            all_boxes.append(Polygon(corners))

			# Paint box number
            cv2.putText(dst,str(len(all_boxes)-1), (int(all_boxes[-1].centroid.coords[0][0]), int(all_boxes[-1].centroid.coords[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (128,0,0), 1)

			# Save also corners positions
            cells_unit.append(pts)

        py += H

    return all_boxes, cells_unit

# Transform x,y coordinates into cell coordinates
# particles positions are in pos_time[particle_id][property]
# where property is: 0: time, 1: x, 2: y, 3: cell
def trajectories_with_indexes():
	for p in range(len(pos_time)):
		for t in range(0, len(pos_time[p][0])):
			points = [pos_time[p][1][t], pos_time[p][2][t]]
			found = False

			# You can speed up this process if you look for neighbourhood cells instead of all of them.
			# That would depend on how your cells are in the image.
			boxes = cells_unit_shapely
			boxes_num = range(0,len(all_boxes)-1)


			for i in range(len(boxes)):
				if Point(points).intersects(boxes[i]) == True:
					g.pos_time[p][3][t] = boxes_num[i]
					found = True
					break
			if found == False:
            	g.pos_time[p][3][t] = -1


# Control boxes with keyboard.
# I only include interaction to move the box to the left by pressing a
# DO the same for other directions.
def keyboard_control_boxes(dst, ms, show):
	global ind_cell, cells_unit

    # Init variables
    finish = True
    dst_tmp = dst.copy()

    # Do always
    while(finish):
        # Show frame if asked
        if show == True:
            cv2.imshow("Window",dst)

        k = cv2.waitKey(ms) & 0xFF

        if k == ord('q') or k == ord('Q'):			#Quit frame
            return 0
        elif k == ord('a') or k == ord('A'):		#ASWD: Typical keys to move the position of the particle         
            if ind_cell > -1:
				for i in range(len(cells_unit[ind_cell])):
					cells_unit[ind_cell][i][0] += -1
					cells_unit[ind_cell][i][0] += -1
    	        cells_unit_shapely[ind_cell] = Polygon(cells_unit[ind_cell])

            dst = frame.copy()					
            draw_only_boxes_lines(dst)
        elif k == ord('z') or k == ord('Z'):		#Move to the next or previous particle in this class
            if ind_cell > 0:
                ind_cell -= 1
						
            dst = frame.copy()			

# Draw only boxes from previous definition
# Put in a different colour the box selected
def draw_only_boxes_lines(dst):
    for i in range(len(cells_unit)):
        pts = cells_unit[i]
        pts = pts.reshape((-1,1,2))
        cv2.polylines(dst,[pts],True,(0,0,255))

    if ind_cell > -1 and ind_cell < len(all_boxes):
        pts = cells_unit[g.ind_cell]
        pts = pts.reshape((-1,1,2))
        cv2.polylines(dst,[pts],True,(128,128,0))


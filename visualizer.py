# Author: Shing-Yan Loo (lsyan@ualberta.ca)
# Visualize the sensor readings

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec

import time

from numpy.core.shape_base import block

import cv2

def visualizer(visual_msg_queue, quit):
    # Setup layout
    fig, axs = plt.subplots() #1, 2)

    # Expand the plot when needed
    plt.autoscale()
    
    # animated=True tells matplotlib to only draw the artist when we
    # explicitly request it
    # (ln,) = axs.plot([0], [0]) #, animated=True)

    # make sure the window is raised, but the script keeps going
    plt.show(block=False)

    # # get copy of entire figure (everything inside fig.bbox) sans animated artist
    # bg = fig.canvas.copy_from_bbox(fig.bbox)
    # # draw the animated artist, this uses a cached renderer
    # axs.draw_artist(ln)

    while not quit.value or not visual_msg_queue.empty():
        if visual_msg_queue.empty():
            time.sleep(0.01)
            continue
        print('refreshing visualizer....')
        fig.canvas.draw()
        # reset the background back in the canvas state, screen unchanged
        # fig.canvas.restore_region(bg)

        msg = visual_msg_queue.get()

        if msg.get('gt_traj') is not None:
            gt_traj = msg['gt_traj']
            # update the artist, neither the canvas state nor the screen have changed
            # ln.set_xdata(gt_traj[0])
            # ln.set_xdata(gt_traj[1])

            print('x', len(gt_traj[0]))
            print('y', len(gt_traj[1]))

            # re-render the artist, updating the canvas state, but not the screen
            # axs.draw_artist(ln)
            axs.plot(gt_traj[0], gt_traj[1], color='green', linestyle='solid') #, animated=True)
            # copy the image to the GUI state, but screen might not be changed yet
            fig.canvas.blit(fig.bbox)
            # print('gt_traj', gt_traj)
            # axs[0, 0].scatter(gt_traj[0], gt_traj[1])
        
        # flush any pending GUI events, re-painting the screen if needed
        fig.canvas.flush_events()

# class Visualizer:
#     def __init__(self, visual_msg_queue):
#         # # Setup layout
#         # self.fig = plt.figure(figsize=(16,9), dpi=120, facecolor=(0.9, 0.9, 0.9))
#         # gs = gridspec.GridSpec(2, 2)
#         # self.image_panel = self.fig.add_subplot(gs[0, 0])
#         # self.image_panel.set_title('Image')
#         # self.fig.show()

#         # # Data to visualize
#         # self.image = None

#         self.visual_msg_queue = visual_msg_queue


#     # def add_msg(self, msg):
#     #     """Add new information to the visualization (image, sensor readings
#     #     pose estimation, etc.)

#     #     :param msg: a dict containing the visualization information 
#     #         (image, sensor readings, etc.)
#     #     :type msg: dict
#     #     """
#     #     self.fig.canvas.draw()

#     #     if msg.get('image') is not None:
#     #         self.image = msg['image']
#     #         print('refreshing image')
#     #         self.image_panel.imshow(self.image)
#     #         self.update_image.value = True
            

#     # def animate(self, i):
#     #     print('refreshing visualization', i)
#     #     if self.image is not None:
            

#     def refresh(self, quit):
        # # Setup layout
        # # fig = plt.figure(figsize=(16,9), dpi=120, facecolor=(0.9, 0.9, 0.9))
        # # gs = gridspec.GridSpec(3, 2)
        # # ax_imu = fig.add_subplot(gs[:2, 0])
        
        # # # make sure the window is raised, but the script keeps going
        # # plt.show(block=False)

        # while not quit.value:
        #     if self.visual_msg_queue.empty():
        #         time.sleep(0.01)
        #         continue
        #     print('refreshing visualizer....')
        #     # fig.canvas.draw()

        #     msg = self.visual_msg_queue.get()

        #     # if msg.get('image') is not None:
        #     #     image = msg['image']
        #     #     # print('refreshing image')
        #     #     # image_panel.imshow(image, animated=True)
        #     #     # fig.canvas.blit(image_panel.bbox)
        #     #     cv2.imshow('image', image)
        #     #     cv2.waitKey(1)

        #     time.sleep(0.01)

        #     # fig.canvas.flush_events()


    # def start(self):
    #     # Run animation in the background (refresh every 10 ms)
    #     print('starting visualization')
    #     ani = animation.FuncAnimation(self.fig, self.animate, interval=10)
    #     plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)

# def animate(i):
#     with open('example.txt', 'r') as f:
#         lines = f.read().split('\n')

#         xs = []
#         ys = []
#         for line in lines:
#             x, y = line.split(',')
#             xs.append(float(x))
#             ys.append(float(y))
        
#         ax.clear()
#         ax.plot(xs, ys)

#         print(xs, ys)

# ani = animation.FuncAnimation(fig, animate, interval=1000)
# plt.axis([0, 10, 0, 12])
# plt.show()
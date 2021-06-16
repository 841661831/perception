import open3d as o3d
import numpy as np
from src import data_extract as corner2box


def custom_draw_geometry(pcd,linesets_py, linesets_cpp, linesets_kd):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    for i in linesets_py:
        vis.add_geometry(i)
    for i in linesets_cpp:
        vis.add_geometry(i)
    for i in linesets_kd:
        vis.add_geometry(i)
    render_option = vis.get_render_option()
    render_option.point_size = 1
    render_option.background_color = np.asarray([0, 0, 0])
    vis.run()
    vis.destroy_window()

def view_res(view_range):
    '''  '''
    point_cloud = o3d.geometry.PointCloud()
    for i in range(21):
        points = np.load("/data/second_cpp2/datasets/kitti/0000" + str(i).zfill(2) + ".npy");
        bbox_cpp = np.load("/data/second_cpp2/datasets/pre_res/0000" + str(i).zfill(2) + "_cpp.npy")
        bbox_py = np.load("/data/second_cpp2/datasets/pre_res/0000" + str(i).zfill(2) + "_py.npy")
        bbox_kd = np.load("/data/second_cpp2/datasets/pre_res/0000" + str(i).zfill(2) + "_cpp_kd.npy")
        print("the {} frame :  python has {} boxes  cpp has {} boxes  kd has {} boxes".format(i, bbox_py.shape[0], bbox_cpp.shape[0], bbox_kd.shape[0]))
        if view_range:
            points_range = []
            for i in range(len(points)):
                x, y,z, _ = points[i]
                if 0 <= x <= 52.8 and  -32.0 <= y <= 32.0 and -3 <= z <= 1:
                    points_range.append(points[i])
            points_range = np.array(points_range).reshape((-1, 4))
            points = points_range
        line_sets_cpp = [o3d.geometry.LineSet() for _ in range(len(bbox_cpp))]
        line_sets_py = [o3d.geometry.LineSet() for _ in range(len(bbox_py))]
        line_sets_kd = [o3d.geometry.LineSet() for _ in range(len(bbox_kd))]
        boxes_corner_cpp = corner2box.center_to_corner_box3d(
            bbox_cpp[:, :3],
            bbox_cpp[:, 3:6],
            bbox_cpp[:, 6])
        boxes_corner_py = corner2box.center_to_corner_box3d(
            bbox_py[:, :3],
            bbox_py[:, 3:6],
            bbox_py[:, 6])
        boxes_corner_kd = corner2box.center_to_corner_box3d(
            bbox_kd[:, :3],
            bbox_kd[:, 3:6],
            bbox_kd[:, 6])
        # boxes_corner_py += 0.1
        for i in range(len(boxes_corner_cpp)):
            points_box = boxes_corner_cpp[i]
            lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7],
                                  [0, 4], [1, 5], [2, 6], [3, 7]])
            colors = np.array([[0, 1, 0] for j in range(len(lines_box))])
            line_sets_cpp[i].points = o3d.utility.Vector3dVector(points_box)
            line_sets_cpp[i].lines = o3d.utility.Vector2iVector(lines_box)
            line_sets_cpp[i].colors = o3d.utility.Vector3dVector(colors)
        for i in range(len(boxes_corner_py)):
            points_box = boxes_corner_py[i]
            lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7],
                                  [0, 4], [1, 5], [2, 6], [3, 7]])
            colors = np.array([[1, 0, 0] for j in range(len(lines_box))])
            line_sets_py[i].points = o3d.utility.Vector3dVector(points_box)
            line_sets_py[i].lines = o3d.utility.Vector2iVector(lines_box)
            line_sets_py[i].colors = o3d.utility.Vector3dVector(colors)
        for i in range(len(boxes_corner_kd)):
            points_box = boxes_corner_kd[i]
            lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7],
                                  [0, 4], [1, 5], [2, 6], [3, 7]])
            colors = np.array([[1, 1, 1] for j in range(len(lines_box))])
            line_sets_kd[i].points = o3d.utility.Vector3dVector(points_box)
            line_sets_kd[i].lines = o3d.utility.Vector2iVector(lines_box)
            line_sets_kd[i].colors = o3d.utility.Vector3dVector(colors)
        point_cloud.points = o3d.utility.Vector3dVector(points[:, :3])
        custom_draw_geometry(point_cloud, line_sets_py, line_sets_cpp, line_sets_kd)


if __name__ == "__main__":
    view_res(True)







import os
import sys
sys.path.append("..")
import numpy as np
from sklearn.decomposition import PCA
from sklearn import preprocessing
import flatbuffers
import morphable_model.core.PointCloudR as PointCloudR
import morphable_model.core.PcaModelR as PcaModelR

class MHDataset:

    def __init__(self, n_points_in_sample=14444):
        self.n_points_in_sample = 14444
        self.data = np.zeros([1, 1])

    def load_dataset(self, mhsamples_file_path, max_line=-1):
        print("MHDataset - load_dataset - ", mhsamples_file_path)
        lines = [line.strip() for line in open(mhsamples_file_path)]
        cur_file = 0
        if max_line == -1:
            max_line = len(lines) - 1
        self.data = np.zeros([max_line, self.n_points_in_sample * 3])

        for idx, line in enumerate(lines):
            if idx == 0:
                continue
            if cur_file >= max_line:
                break
            split_line = line.split(" ")
            points = self.parse_pcd(split_line[-1])
            cur_data = np.zeros([len(points), 3])
            for idx_data, point in enumerate(points):
                x, y, z = point.split(" ")
                cur_data[idx_data][0] = float(x)
                cur_data[idx_data][1] = float(y)
                cur_data[idx_data][2] = float(z)
            self.data[cur_file, :] = cur_data.reshape([1, cur_data.size])
            cur_file += 1
        print("data", self.data.shape)
        print(self.data[0, 0:10])
        print(self.data[-1, 0:10])

    def parse_pcd(self, pcd_file_path):
        # TODO: support more pcd types
        points = []
        lines = [line.strip() for line in open(pcd_file_path)]
        for idx, line in enumerate(lines):
            if idx <= 10:
                continue
            points.append(line)
        return points

    def get_data(self):
        pass

    def load_dataset_flatbuffers(self, mhsamples_file_path, max_line=-1):
        print("MHDataset - load_dataset - ", mhsamples_file_path)
        lines = [line.strip() for line in open(mhsamples_file_path)]
        cur_file = 0
        if max_line == -1:
            max_line = len(lines) - 1
        self.data = np.zeros([max_line, self.n_points_in_sample * 3])

        for idx, line in enumerate(lines):
            if idx == 0:
                continue
            if cur_file >= max_line:
                break
            split_line = line.split(" ")

            file = open(split_line[-1] + "_fb.bin", "rb")
            data = file.read()
            cloud = PointCloudR.PointCloudR.GetRootAsPointCloudR(data, 0)
            self.data[cur_file, :] = cloud.PointsAsNumpy()
            cur_file += 1
        print("data", self.data.shape)
        print(self.data[0, 0:10])
        print(self.data[-1, 0:10])


if __name__ == '__main__':
    import win_unicode_console
    win_unicode_console.enable()
    from datetime import datetime
    dataset = MHDataset()
    max_line = -1
    t1 = datetime.now()
    dataset.load_dataset_flatbuffers("D:\\mhmodels\\dataset.txt", max_line)
    print("Time to load dataset: ", (datetime.now() - t1).seconds, " seconds")
    print("Dataset size: ", dataset.data.shape)

    scaler = preprocessing.StandardScaler()
    data_normalized = scaler.fit_transform(dataset.data)

    pca = PCA(n_components=5, svd_solver='randomized')
    t1 = datetime.now()
    new_data = pca.fit_transform(data_normalized)
    print("Time to solve pca: ", (datetime.now() - t1).seconds, " seconds")
    print("components_.shape", pca.components_.shape)
    print("pca.components_", pca.components_)
    print("pca.singular", pca.singular_values_, np.linalg.norm(pca.components_))
    print("pca.noise_variance_", pca.noise_variance_)
    print("pca.explained_variance", pca.explained_variance_)
    print("pca.explained_variance_ratio", pca.explained_variance_ratio_)
    print("pca.mean_", pca.mean_)
    print("pca.mean_.shape", pca.mean_.shape)
    new_shape = pca.inverse_transform([0, -20, 0, 0, 0])
    # inverse scale
    # new_shape = scaler.inverse_transform(new_shape)
    new_shape = scaler.mean_ + new_shape * scaler.scale_
    print("scaler.mean", scaler.mean_.shape)
    print("scaler.scale_", scaler.scale_.shape)

    reshape_data = np.reshape(new_shape, [dataset.n_points_in_sample, 3])
    print("reshape_data", reshape_data)
    file = open("D:\\test.pcd", "w")
    file.write("""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 14444
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 14444
DATA ascii\n""")
    for i in range(dataset.n_points_in_sample):
        file.write("{} {} {}\n".format(reshape_data[i][0], reshape_data[i][1], reshape_data[i][2]))

    # Create PCAModelR
    n_features = pca.n_features_
    n_components = pca.n_components_
    builder = flatbuffers.Builder(0)

    PcaModelR.PcaModelRStartScalerMeanVector(builder, scaler.mean_.size)
    for v in scaler.mean_[::-1]:
        # Get from right to left because we will use PREPEND instead of APPEND
        builder.PrependFloat32(v)
    pca_scaler_mean = builder.EndVector(scaler.mean_.size)

    PcaModelR.PcaModelRStartScalerScaleVector(builder, scaler.scale_.size)
    for v in scaler.scale_[::-1]:
        # Get from right to left because we will use PREPEND instead of APPEND
        builder.PrependFloat32(v)
    pca_scaler_scale = builder.EndVector(scaler.scale_.size)

    PcaModelR.PcaModelRStartMeanVector(builder, pca.mean_.size)
    for v in pca.mean_[::-1]:
        # Get from right to left because we will use PREPEND instead of APPEND
        builder.PrependFloat32(v)
    pca_mean = builder.EndVector(pca.mean_.size)

    reshape_component = np.reshape(pca.components_, [-1, 1])
    PcaModelR.PcaModelRStartComponentsVector(builder, reshape_component.size)
    for v in reshape_component[::-1]:
        # Get from right to left because we will use PREPEND instead of APPEND
        builder.PrependFloat32(v)
    pca_component = builder.EndVector(reshape_component.size)

    explained_variance_size = pca.explained_variance_.size
    PcaModelR.PcaModelRStartExplainedVarianceVector(builder, explained_variance_size)
    for v in pca.explained_variance_[::-1]:
        # Get from right to left because we will use PREPEND instead of APPEND
        builder.PrependFloat32(v)
    pca_variance = builder.EndVector(explained_variance_size)

    PcaModelR.PcaModelRStart(builder)
    PcaModelR.PcaModelRAddScalerMean(builder, pca_scaler_mean)
    PcaModelR.PcaModelRAddScalerScale(builder, pca_scaler_scale)
    PcaModelR.PcaModelRAddMean(builder, pca_mean)
    PcaModelR.PcaModelRAddExplainedVariance(builder, pca_variance)
    PcaModelR.PcaModelRAddComponents(builder, pca_component)
    PcaModelR.PcaModelRAddNFeatures(builder, n_features)
    PcaModelR.PcaModelRAddNComponents(builder, n_components)
    pca = PcaModelR.PcaModelREnd(builder)
    builder.Finish(pca)
    buf = builder.Output()

    file = open("D:\\pca_model.bin", "wb")
    file.write(buf)
    file.close()
    print("buf length", len(buf))




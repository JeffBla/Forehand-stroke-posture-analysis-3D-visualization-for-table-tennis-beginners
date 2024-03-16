import sys

sys.path.append(
    '/home/jeffbla/Project/bvh_analysis_viewer_withEngine/Py_package/lstm_posture_suggestion/forehand'
)

import numpy as np
import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt

from tslearn.metrics import lcss_path
from tslearn.utils import to_time_series_dataset
from tslearn.preprocessing import TimeSeriesResampler, TimeSeriesScalerMeanVariance

from multipledispatch import dispatch

from base_lstm import LSTMClassifier

#################### CONSTANT ####################

target_ref_table = {
    "rForeArm": "rForeArm_parent",
    "lForeArm": "lForeArm_parent",
}

FOREARM_DEVIATE = 1
STD_FOREARM_AMPLITUDE = 3

#################### Function ####################


def WaistRotationRead(filename: str):
    df = pd.read_csv(filename)

    df_rshoulder_x = df['2DX_rshoulder']

    df_lshoulder_x = df['2DX_lshoulder']

    df_dist = pd.DataFrame(np.abs(df_rshoulder_x - df_lshoulder_x),
                           columns=["dist_y"])

    df_dist["std"] = (df_dist["dist_y"] -
                      df_dist["dist_y"].mean()) / df_dist["dist_y"].std()
    return df_dist["std"].values


@dispatch(str, str)
def WaistRotationPreprocess(target_filename: str, ref_filename: str):
    df_target = WaistRotationRead(target_filename)
    df_ref = WaistRotationRead(ref_filename)

    return WaistRotationPreprocess(df_target, df_ref)


@dispatch(np.ndarray, np.ndarray)
def WaistRotationPreprocess(df_target: np.ndarray, df_ref: np.ndarray):
    num_frame = max(len(df_target), len(df_ref))

    # Resample the time series
    df_target = TimeSeriesResampler(sz=num_frame).fit_transform(df_target)
    df_ref = TimeSeriesResampler(sz=num_frame).fit_transform(df_ref)

    return df_target, df_ref


@dispatch(str, str)
def OpenPoseAnalysize_Waist(target_filename: str, ref_filename: str):
    df_target, df_ref = WaistRotationPreprocess(target_filename, ref_filename)

    return OpenPoseAnalysize_Waist(df_target, df_ref)


@dispatch(np.ndarray, np.ndarray)
def OpenPoseAnalysize_Waist(df_target: np.ndarray, df_ref: np.ndarray):
    dataset = np.concatenate([df_target, df_ref])

    # Compute the LCSS
    path, sim = lcss_path(dataset[0, :, 0], dataset[1, :, 0], eps=0.5)
    return sim


@dispatch(str, str, str)
def BvhAnalyzePreprocess(target_filename: str, ref_filename: str, target: str):
    df_target = pd.read_csv(target_filename)
    df_ref = pd.read_csv(ref_filename)

    return BvhAnalyzePreprocess(df_target, df_ref, target)


@dispatch(pd.DataFrame, pd.DataFrame, str)
def BvhAnalyzePreprocess(df_target: pd.DataFrame, df_ref: pd.DataFrame,
                         target: str):
    num_frame = max(df_target.count().max(), df_ref.count().max())

    scaler = TimeSeriesScalerMeanVariance(mu=0, std=1)
    df_target_result = pd.DataFrame()
    df_ref_result = pd.DataFrame()
    for column in df_target.columns:
        if target in column:
            # Rescale & Resample the time series
            ts1 = TimeSeriesResampler(sz=num_frame).fit_transform(
                df_target[column].values)
            ts2 = TimeSeriesResampler(sz=num_frame).fit_transform(
                df_ref[column].values)
            ts1 = scaler.fit_transform(ts1)
            ts2 = scaler.fit_transform(ts2)
            df_target_result[column] = ts1.reshape(-1)
            df_ref_result[column] = ts2.reshape(-1)

    return df_target_result, df_ref_result


@dispatch(str, str, str)
def BvhAnalyze(target_filename: str, ref_filename: str, target: str) -> dict:
    df_target, df_ref = BvhAnalyzePreprocess(target_filename, ref_filename,
                                             target)

    return BvhAnalyze(df_target, df_ref, target)


@dispatch(pd.DataFrame, pd.DataFrame, str)
def BvhAnalyze(df_target: pd.DataFrame, df_ref: pd.DataFrame,
               target: str) -> dict:
    simList = {}
    # Find the target column
    for column in df_target.columns:
        if target in column:
            dataset = to_time_series_dataset(
                [df_target[column].values, df_ref[column].values])

            # Compute the LCSS
            path, sim = lcss_path(dataset[0, :, 0], dataset[1, :, 0], eps=0.5)
            simList[column] = sim
    return simList


def ForehandStrokeAnalysis(openpose_target_filename: str,
                           bvh_target_filename: str) -> np.array:
    PROB_THRESHOLD = 0.5

    annotation_file = pd.DataFrame(
        [[openpose_target_filename, bvh_target_filename]],
        columns=["openpose_files", "angle_files"])
    annotation_file.to_csv("tmp.csv", index=False)

    pred = LSTMClassifier.lstm_forehand_predict(
        "tmp.csv",
        "~/Project/bvh_analysis_viewer_withEngine/Py_package/lstm_posture_suggestion/forehand/pth/test/resnet-epoch=606-train_loss=0.0000.ckpt",
        batch_size=1)
    result = pred[0][0].tolist()
    return result


if __name__ == "__main__":
    # print(
    #     OpenPoseAnalysize_Waist("cmake-build-debug/output/openposeTest.csv",
    #                             "cmake-build-debug/output/openposeRef.csv"))
    # print(
    #     BvhAnalyze("cmake-build-debug/output/whole_body.csv",
    #                "cmake-build-debug/output/me/whole_body.csv", "hip"))

    ForehandStrokeAnalysis(
        "/home/jeffbla/Video/bvh_analysis/output/openposeTest.csv",
        "cmake-build-debug/output/forehand_stroke.csv")

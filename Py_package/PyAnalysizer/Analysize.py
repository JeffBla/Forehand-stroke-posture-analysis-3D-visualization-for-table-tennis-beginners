import numpy as np
import pandas as pd

from tslearn.metrics import lcss_path
from tslearn.utils import to_time_series_dataset
from tslearn.preprocessing import TimeSeriesResampler, TimeSeriesScalerMeanVariance


def AnalyszeWaistRotation(filename: str):
    df = pd.read_csv(filename)

    df_rshoulder_x = df['2DX_rshoulder']

    df_lshoulder_x = df['2DX_lshoulder']

    df_dist = pd.DataFrame(np.abs(df_rshoulder_x - df_lshoulder_x),
                           columns=["dist_y"])

    df_dist["std"] = (df_dist["dist_y"] -
                      df_dist["dist_y"].mean()) / df_dist["dist_y"].std()
    return df_dist


def OpenPoseAnalysize_Waist(target_filename: str, ref_filename: str):
    df_target = AnalyszeWaistRotation(target_filename)
    df_ref = AnalyszeWaistRotation(ref_filename)

    num_frame = max(df_target.count().max(), df_ref.count().max())

    # Resample the time series
    dataset = to_time_series_dataset(
        [df_target["std"].values, df_ref["std"].values])
    dataset = TimeSeriesResampler(sz=num_frame).fit_transform(dataset)

    # Compute the LCSS
    path, sim = lcss_path(dataset[0, :, 0], dataset[1, :, 0], eps=0.5)

    return sim


def BvhAnalysize(target_filename: str, ref_filename: str, target: str) -> dict:
    df_target = pd.read_csv(target_filename)
    df_ref = pd.read_csv(ref_filename)

    num_frame = max(df_target.count().max(), df_ref.count().max())

    scaler = TimeSeriesScalerMeanVariance(mu=0, std=1)
    simList = {}
    # Find the target column
    for column in df_target.columns:
        if target in column:
            # Rescale & Resample the time series
            ts1 = TimeSeriesResampler(sz=num_frame).fit_transform(
                df_target[column].values)
            ts2 = TimeSeriesResampler(sz=num_frame).fit_transform(
                df_ref[column].values)
            dataset = scaler.fit_transform(np.concatenate([ts1, ts2]))

            # Compute the LCSS
            path, sim = lcss_path(dataset[0, :, 0], dataset[1, :, 0], eps=0.5)
            simList[column] = sim
    return simList


if __name__ == "__main__":
    print(
        OpenPoseAnalysize_Waist("cmake-build-debug/output/openposeTest.csv",
                                "cmake-build-debug/output/openposeRef.csv"))
    print(
        BvhAnalysize("cmake-build-debug/output/whole_body.csv",
                     "cmake-build-debug/output/me/whole_body.csv", "hip"))

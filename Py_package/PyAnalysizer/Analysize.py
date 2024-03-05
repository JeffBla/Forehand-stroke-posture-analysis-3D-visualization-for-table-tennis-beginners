import numpy as np
import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt

from tslearn.metrics import lcss_path
from tslearn.utils import to_time_series_dataset
from tslearn.preprocessing import TimeSeriesResampler, TimeSeriesScalerMeanVariance

from multipledispatch import dispatch

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
    return df_dist


@dispatch(str, str)
def WaistRotationPreprocess(target_filename: str, ref_filename: str):
    df_target = WaistRotationRead(target_filename)
    df_ref = WaistRotationRead(ref_filename)

    return WaistRotationPreprocess(df_target, df_ref)


@dispatch(pd.DataFrame, pd.DataFrame)
def WaistRotationPreprocess(df_target: pd.DataFrame, df_ref: pd.DataFrame):
    num_frame = max(df_target.count().max(), df_ref.count().max())

    # Resample the time series
    df_target = TimeSeriesResampler(sz=num_frame).fit_transform(df_target)
    df_ref = TimeSeriesResampler(sz=num_frame).fit_transform(df_ref)

    return df_target, df_ref


@dispatch(str, str)
def OpenPoseAnalysize_Waist(target_filename: str, ref_filename: str):
    df_target, df_ref = WaistRotationPreprocess(target_filename, ref_filename)

    return OpenPoseAnalysize_Waist(df_target, df_ref)


@dispatch(pd.DataFrame, pd.DataFrame)
def OpenPoseAnalysize_Waist(df_target: pd.DataFrame, df_ref: pd.DataFrame):
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
                           openpose_ref_filename: str,
                           bvh_target_filename: str, bvh_ref_filename: str,
                           target: str) -> bool:
    df_openpose_target, df_openpose_ref = WaistRotationPreprocess(
        openpose_target_filename, openpose_ref_filename)

    df_target, df_ref = BvhAnalyzePreprocess(bvh_target_filename,
                                             bvh_ref_filename, target)

    if target == "hip":
        score = OpenPoseAnalysize_Waist(df_openpose_target, df_openpose_ref)
    else:
        score = BvhAnalyze(df_target, df_ref, target)
        score = score[target_ref_table[target]]

        # Visualize the result
        # plt.figure()
        # plt.plot(df_target[target_ref_table[target]], "b-", label="target")
        # plt.plot(df_ref[target_ref_table[target]], "r-", label="ref")

        # plt.legend()
        # plt.show()
    if score > 0.5:

        return True
    else:
        # if target == "hip":
        #     # Analyze amplitude

        #     return ""
        # elif target == "rForeArm":
        #     # Analyze amplitude & U
        #     crests = signal.argrelextrema(
        #         df_target[target_ref_table[target]].values,
        #         np.greater,
        #         order=3)[0]
        #     # Add the first and last index
        #     crests = np.append(np.insert(crests, 0, 0),
        #                        df_target.count().max() - 1)
        #     throughs = signal.argrelextrema(
        #         df_target[target_ref_table[target]].values, np.less,
        #         order=3)[0]
        #     # Add the first and last index
        #     throughs = np.append(np.insert(throughs, 0, 0),
        #                          df_target.count().max() - 1)

        #     # Find two crest points that are highest
        #     crest_idx1 = -1
        #     crest_idx2 = -1
        #     for idx in crests:
        #         if df_target[
        #                 target_ref_table[target]].values[idx] > crest_idx1:
        #             crest_idx2 = crest_idx1
        #             crest_idx1 = idx
        #         elif df_target[
        #                 target_ref_table[target]].values[idx] > crest_idx2:
        #             crest_idx2 = idx
        #     # Find a througn point that is lowest
        #     throughs.clip(crest_idx1, crest_idx2, out=throughs)
        #     throughs = np.unique(throughs)
        #     argmin_throughs = df_target[
        #         target_ref_table[target]].iloc[throughs].argmin()
        #     through_idx = throughs[argmin_throughs]

        #     # Calculate the amplitude and the y distance between the two crest points
        #     amplitude = df_target[
        #         target_ref_table[target]].values[crest_idx2] - df_target[
        #             target_ref_table[target]].values[through_idx]
        #     y_dist_crest = df_target[
        #         target_ref_table[target]].values[crest_idx1] - df_target[
        #             target_ref_table[target]].values[crest_idx2]
        #     if amplitude > STD_FOREARM_AMPLITUDE + FOREARM_DEVIATE:
        #         return "You should wave your arm more gently."
        #     elif amplitude < STD_FOREARM_AMPLITUDE - FOREARM_DEVIATE:
        #         return "You should wave your arm more powerfully."
        #     return ""
        return False


if __name__ == "__main__":
    # print(
    #     OpenPoseAnalysize_Waist("cmake-build-debug/output/openposeTest.csv",
    #                             "cmake-build-debug/output/openposeRef.csv"))
    # print(
    #     BvhAnalyze("cmake-build-debug/output/whole_body.csv",
    #                "cmake-build-debug/output/me/whole_body.csv", "hip"))

    ForehandStrokeAnalysis("cmake-build-debug/output/openposeTest.csv",
                           "cmake-build-debug/output/openposeRef.csv",
                           "cmake-build-debug/output/whole_body.csv",
                           "cmake-build-debug/output/me/whole_body.csv",
                           "rForeArm")

from collections import deque
import time

import numpy as np
import VariableConfig

from DualLivePrediction import (
    load_model,
    read_live_sample,
    calculate_actual_positions,
    predict_positions,
    DelayedPredictionEvaluator,
)


# --------------------------- Configuration ---------------------------

model_filename = VariableConfig.MODEL_FILENAME
sample_shift = VariableConfig.SAMPLE_SHIFT
imu_features = VariableConfig.IMU_FEATURES
target_features = VariableConfig.TARGET_FEATURES

REPORT_EVERY = 100

# Set to 1 / your actual sensor update rate if you want to pace the loop.
# Leave as None if the Xsens acquisition function already controls timing.
EXPECTED_SAMPLE_PERIOD_S = None


# --------------------------- Validation ---------------------------

if sample_shift < 1:
    raise ValueError("SAMPLE_SHIFT must be at least 1.")

if REPORT_EVERY < 1:
    raise ValueError("REPORT_EVERY must be at least 1.")

if len(imu_features) != 36:
    raise ValueError(
        f"Expected 36 IMU input features, got {len(imu_features)}."
    )

if len(target_features) != 18:
    raise ValueError(
        f"Expected 18 Cartesian position outputs, got {len(target_features)}."
    )


# --------------------------- Load model ---------------------------

model = load_model(model_filename)

print(f"Loaded model: {model_filename}")
print(f"IMU inputs: {len(imu_features)}")
print(f"Position outputs: {len(target_features)}")
print(f"Prediction horizon: {sample_shift} samples")
print("Evaluation only: robot movement is disabled.")
print("Press Ctrl+C to stop.\n")


# --------------------------- Evaluator setup ---------------------------

evaluator = DelayedPredictionEvaluator(
    sample_shift=sample_shift,
    target_features=target_features,
)

sample_index = 0


# --------------------------- Main loop ---------------------------

try:
    while True:
        loop_start = time.perf_counter()

        # 1. Read one synchronized live Xsens sample.
        sample = read_live_sample()

        # 2. Calculate actual Cartesian positions from this sample.
        actual_positions = calculate_actual_positions(sample)

        # 3. Compare a prediction made SAMPLE_SHIFT samples ago.
        result = evaluator.compare_if_due(actual_positions)

        if result is not None:

            if not result["matched"]:
                print(
                    f"WARNING: Missed target sample "
                    f"{result['target_sample']}; "
                    f"current sample is {sample_index}."
                )

            else:
                print(
                    f"Sample={sample_index} | "
                    f"Predicted at sample={result['source_sample']} | "
                    f"Mean of six point errors="
                    f"{result['mean_point_error']:.5f} m"
                )

                if evaluator.matched_count % REPORT_EVERY == 0:
                    evaluator.print_rmse()

        # 4. Predict future positions from the current IMU sample.
        prediction = predict_positions(
            model=model,
            imu_sample=sample,
            imu_features=imu_features,
            target_features=target_features,
        )

        # 5. Store the prediction until its target sample arrives.
        evaluator.add_prediction(prediction)

        # 6. Advance the sample counter exactly once per sample.
        evaluator.increment_sample()
        sample_index += 1

        # Optional loop pacing.
        if EXPECTED_SAMPLE_PERIOD_S is not None:
            elapsed = time.perf_counter() - loop_start
            remaining = EXPECTED_SAMPLE_PERIOD_S - elapsed

            if remaining > 0:
                time.sleep(remaining)

except KeyboardInterrupt:
    print("\nStopping live prediction evaluation...")

finally:
    print("\n========== FINAL RESULTS ==========")

    if evaluator.matched_count > 0:
        evaluator.print_rmse()
    else:
        print("No predictions were scored.")
        print(
            "Check the Xsens data stream, actual-position calculation, "
            "and that the stream ran for at least SAMPLE_SHIFT + 1 samples."
        )

    print(f"Total live samples processed: {sample_index}")
    print(f"Matched predictions scored: {evaluator.matched_count}")
    print("Robot movement was not enabled.")


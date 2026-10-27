import joblib
import copy

# Models to combine
model_files = [
    "dual_human_joint_coordinates_predictor_3d1.pkl",
    "dual_human_joint_coordinates_predictor_3d2.pkl",
    "dual_human_joint_coordinates_predictor_3d3.pkl",
    "dual_human_joint_coordinates_predictor_3d4.pkl",
    "dual_human_joint_coordinates_predictor_3d5.pkl",
    "dual_human_joint_coordinates_predictor_3d6.pkl"
]

# Load all models
models = [joblib.load(file) for file in model_files]

# Use first model as the base
combined_model = copy.deepcopy(models[0])

# Combine decision trees from every model
combined_model.estimators_ = [
    tree
    for model in models
    for tree in model.estimators_
]

# Update number of trees
combined_model.n_estimators = len(combined_model.estimators_)

# Save combined model
joblib.dump(combined_model, "combined_RF_model.pkl")

print(f"Combined {len(models)} Random Forest models")
print(f"Total trees: {combined_model.n_estimators}")
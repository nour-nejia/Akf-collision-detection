
import pandas as pd
import numpy as np
import joblib, json, matplotlib.pyplot as plt
import os
print("Fichiers sauvegardés dans :", os.getcwd())
from sklearn.ensemble        import RandomForestClassifier
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.preprocessing   import LabelEncoder
from sklearn.metrics         import (classification_report,
                                     confusion_matrix, ConfusionMatrixDisplay)
CSV_FILE = r"C:\Users\MSI\Desktop\Nour_Nejia\RT3\RT3_S2\Signaux_systèmes\TP\TP_KALMAN\Partie_D\classification_dataset.csv"
df = pd.read_csv(CSV_FILE)
df.columns = df.columns.str.strip()
print(f"Shape : {df.shape}")
print(df['label'].value_counts(), "\n")

# FEATURES 
FEATURES = ['d_est_m', 'v_est_ms', 'a_est_ms2', 'v_used_ms', 'TTC_s']
df = df.dropna(subset=FEATURES + ['label'])

X     = df[FEATURES].values
y_raw = df['label'].str.strip().values

le = LabelEncoder()
y  = le.fit_transform(y_raw)

print("Classes encodées :")
for i, c in enumerate(le.classes_): print(f"  {i} -> {c}")


X_tr, X_te, y_tr, y_te = train_test_split(
    X, y, test_size=0.2, random_state=42, stratify=y)

#  MODÈLE 
rf = RandomForestClassifier(
    n_estimators     = 200,
    max_depth        = None,
    min_samples_leaf = 2,
    class_weight     = 'balanced',   # gère déséquilibre
    random_state     = 42,
    n_jobs           = -1
)
rf.fit(X_tr, y_tr)

# ÉVALUATION 
y_pred = rf.predict(X_te)
print(classification_report(y_te, y_pred, target_names=le.classes_))

cv = cross_val_score(rf, X, y, cv=5, scoring='accuracy')
print(f"Cross-val : {cv.mean():.4f} +/- {cv.std():.4f}\n")

# FIGURES 
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

ConfusionMatrixDisplay(confusion_matrix(y_te, y_pred),
                       display_labels=le.classes_).plot(
    ax=axes[0], colorbar=False, cmap='Blues')
axes[0].set_title("Matrice de confusion")
axes[0].tick_params(axis='x', rotation=20)

imp = rf.feature_importances_
axes[1].barh(FEATURES, imp, color='steelblue')
axes[1].set_title("Importance des features")
for i, v in enumerate(imp):
    axes[1].text(v+.002, i, f"{v:.3f}", va='center')

plt.tight_layout()
plt.savefig("rf_evaluation.png", dpi=150)
plt.show()

# SAUVEGARDE PKL + JSON
joblib.dump(rf, "kalman_rf_model.pkl")
json.dump({"features": FEATURES, "classes": list(le.classes_)},
          open("kalman_rf_meta.json","w", encoding="utf-8"), indent=2)
print("Sauvegardé : kalman_rf_model.pkl  +  kalman_rf_meta.json")

# export pour matlab
try:
    import importlib

    convert_sklearn = importlib.import_module("skl2onnx").convert_sklearn
    FloatTensorType = importlib.import_module(
        "skl2onnx.common.data_types"
    ).FloatTensorType

    onnx_model = convert_sklearn(
        rf, initial_types=[('float_input', FloatTensorType([None, len(FEATURES)]))])
    open("kalman_rf_model.onnx","wb").write(onnx_model.SerializeToString())
    print("Export ONNX : kalman_rf_model.onnx  ✓")
except ImportError:
    print("skl2onnx absent -> pip install skl2onnx onnxruntime")

# TEST MANUEL
print("\n--- TEST MANUEL ---")
exemples = [
    [0.04, -0.05, -0.20, -0.04,  0.8],   # collision
    [0.30, -0.12, -0.10, -0.10,  2.5],   # approche
    [0.50,  0.00,  0.00,  0.00, 15.0],   # stationnaire
    [0.80,  0.15,  0.10,  0.14, 15.0],   # éloignement
]
for ex in exemples:
    idx  = rf.predict([ex])[0]
    conf = rf.predict_proba([ex])[0][idx] * 100
    print(f"  {ex}  ->  {le.classes_[idx]:20s}  ({conf:.1f}%)")
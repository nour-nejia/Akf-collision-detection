
import joblib, json, numpy as np, scipy.io as sio

rf      = joblib.load("C:\\Users\\MSI\\Desktop\\Nour_Nejia\\RT3\\RT3 S2\\Signaux et systèmes\\TP\\TP KALMAN\\Partie D\\kalman_rf_model.pkl")
meta    = json.load(open("C:\\Users\\MSI\\Desktop\\Nour_Nejia\\RT3\\RT3 S2\\Signaux et systèmes\\TP\\TP KALMAN\\Partie D\\kalman_rf_meta.json", encoding="utf-8"))
classes = meta["classes"]

n_trees     = len(rf.estimators_)
n_features  = rf.n_features_in_

# Extraire chaque arbre : thresholds, features, valeurs des feuilles
all_children_left   = []
all_children_right  = []
all_feature         = []
all_threshold       = []
all_value           = []

for tree in rf.estimators_:
    t = tree.tree_
    all_children_left .append(t.children_left)
    all_children_right.append(t.children_right)
    all_feature       .append(t.feature)
    all_threshold     .append(t.threshold)
    # value shape: (n_nodes, 1, n_classes) -> (n_nodes, n_classes)
    all_value         .append(t.value[:, 0, :])

# Padder pour avoir la même taille (MATLAB cell array via object array)
max_nodes = max(v.shape[0] for v in all_value)
n_classes = len(classes)

CL  = np.full((n_trees, max_nodes), -999, dtype=np.int32)
CR  = np.full((n_trees, max_nodes), -999, dtype=np.int32)
FT  = np.full((n_trees, max_nodes), -999, dtype=np.int32)
TH  = np.full((n_trees, max_nodes), np.nan)
VAL = np.zeros((n_trees, max_nodes, n_classes))

for i in range(n_trees):
    nn = len(all_children_left[i])
    CL [i, :nn]    = all_children_left[i]
    CR [i, :nn]    = all_children_right[i]
    FT [i, :nn]    = all_feature[i]
    TH [i, :nn]    = all_threshold[i]
    VAL[i, :nn, :] = all_value[i]

sio.savemat("kalman_rf_model.mat", {
    "children_left"  : CL,
    "children_right" : CR,
    "feature"        : FT,
    "threshold"      : TH,
    "value"          : VAL,
    "n_trees"        : n_trees,
    "n_classes"      : n_classes,
    "classes"        : classes
})
print("Sauvegarde : kalman_rf_model.mat")
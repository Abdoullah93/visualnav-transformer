import os
import pickle
import numpy as np

def debug_pkl_files(data_dir):
    """
    Vérifie les fichiers traj_data.pkl dans le dataset pour s'assurer que les données sont homogènes et correctement formatées.
    :param data_dir: Chemin vers le dossier contenant les trajectoires.
    """
    print(f"Débogage des fichiers .pkl dans le dossier : {data_dir}")
    
    # Vérifie si le dossier existe
    if not os.path.exists(data_dir):
        print(f"Erreur : Le dossier {data_dir} n'existe pas.")
        return
    
    # Parcourt les sous-dossiers
    i = 0
    for traj_folder in os.listdir(data_dir):
        if i >= 3:
            break
        i += 1
        traj_path = os.path.join(data_dir, traj_folder)
        pkl_file = os.path.join(traj_path, "traj_data.pkl")
        
        # Vérifie si le fichier traj_data.pkl existe
        if not os.path.exists(pkl_file):
            print(f"Erreur : Le fichier traj_data.pkl est manquant dans {traj_path}.")
            continue
        
        print(f"Vérification du fichier : {pkl_file}")
        
        try:
            # Charge le fichier .pkl
            with open(pkl_file, "rb") as f:
                traj_data = pickle.load(f)
            
            # Vérifie les clés attendues
            expected_keys = ["positions", "yaw", "actions", "goal_pos"]
            for key in expected_keys:
                if key not in traj_data:
                    print(f"Erreur : La clé '{key}' est manquante dans {pkl_file}.")
                else:
                    print(f"Clé '{key}' trouvée.")
            
            # Vérifie les dimensions des données
            positions = traj_data.get("positions", None)
            yaw = traj_data.get("yaw", None)
            
            if positions is not None:
                print(f"positions.shape : {np.shape(positions)} (attendu : [N, 3])")
                if len(np.shape(positions)) != 2 or np.shape(positions)[1] != 3:
                    print(f"Erreur : Les dimensions de 'positions' sont incorrectes dans {pkl_file}.")
            
            if yaw is not None:
                print(f"yaw.shape : {np.shape(yaw)} (attendu : [N])")
                if len(np.shape(yaw)) != 1:
                    print(f"Erreur : Les dimensions de 'yaw' sont incorrectes dans {pkl_file}.")
            
        except Exception as e:
            print(f"Erreur lors du chargement de {pkl_file} : {e}")

# Chemin vers le dossier contenant les trajectoires
# data_dir = "/root/AI-NAVIGATION/visualnav-transformer/train/vint_train/data/data_splits/go_stanford/train/"
data_dir = "/root/AI-NAVIGATION/visualnav-transformer/datasets/go_stanford"
debug_pkl_files(data_dir)
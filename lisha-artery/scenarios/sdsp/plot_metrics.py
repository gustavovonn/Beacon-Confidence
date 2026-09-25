import os
import pandas as pd
import matplotlib.pyplot as plt

# Caminho base relativo à pasta results
BASE_PATH = "../results/"

# Métricas que vamos analisar
METRICS = {
    "auth.csv": "Authentication Latency",
    "ecdh.csv": "ECDH Latency",
    "otp.csv": "OTP Latency",
    "groupKey.csv": "Group Key Latency",
    "aesEnc.csv": "AES Encrypt Latency",
    "aesDec.csv": "AES Decrypt Latency",
    "v2v.csv": "V2V Latency"
}

def load_csv(filename):
    """Carrega um CSV do OMNeT++ exportado pelo scavetool"""
    filepath = os.path.join(BASE_PATH, filename)
    if not os.path.exists(filepath):
        print(f"[ERRO] Arquivo não encontrado: {filepath}")
        return None
    df = pd.read_csv(filepath)
    # Normaliza nomes de colunas (às vezes variam com versão do OMNeT++)
    if "time" in df.columns and "value" in df.columns:
        return df
    elif len(df.columns) >= 2:
        df = df.rename(columns={df.columns[0]: "time", df.columns[1]: "value"})
        return df
    else:
        print(f"[ERRO] Estrutura inesperada em {filename}")
        return None

def plot_metric(df, metric_name, ax):
    """Plota uma métrica em um eixo matplotlib"""
    ax.plot(df["time"], df["value"], label=metric_name, linewidth=1.5)
    ax.set_title(metric_name)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Latência (s)")
    ax.grid(True)

def main():
    fig, axs = plt.subplots(len(METRICS), 1, figsize=(10, 20))
    stats = {}

    for i, (file, metric_name) in enumerate(METRICS.items()):
        df = load_csv(file)
        if df is not None:
            # Estatísticas
            stats[metric_name] = {
                "mean": df["value"].mean(),
                "std": df["value"].std(),
                "min": df["value"].min(),
                "max": df["value"].max()
            }
            # Plot
            plot_metric(df, metric_name, axs[i])

    plt.tight_layout()
    plt.savefig("latency_metrics.png")
    plt.show()

    print("\n==== Estatísticas das Latências ====")
    for metric, values in stats.items():
        print(f"\n{metric}:")
        for k, v in values.items():
            print(f"  {k}: {v:.6f}")

if __name__ == "__main__":
    main()

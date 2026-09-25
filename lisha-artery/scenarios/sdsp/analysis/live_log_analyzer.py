import time
import re
import pandas as pd
import os

# --- Configuração ---
# O script espera estar em uma pasta 'analysis' e o log em uma pasta 'results'
# Ex: ../results/simulation.log
LOG_FILE_PATH = os.path.join("..", "results", "simulation.log")

# Expressão regular para encontrar nossas métricas no log
# Ex: [SDSP_METRIC] authenticationLatency=0.00123
METRIC_REGEX = re.compile(r"\[SDSP_METRIC\]\s+(\w+)=([\d.eE+-]+)")

# --- Início do Script ---

print("--- Monitor de Latência SDSP em Tempo Real ---")
print(f"Observando o arquivo: {LOG_FILE_PATH}\n")

# Dicionário para armazenar todas as latências coletadas
# Ex: {'authenticationLatency': [10.2, 11.5], 'v2vLatency': [22.3]}
collected_data = {}

def process_line(line):
    """
    Processa uma única linha do log, procura pela nossa métrica e a imprime/armazena.
    """
    match = METRIC_REGEX.search(line)
    if match:
        metric_name = match.group(1)
        # Converte o valor de segundos para milissegundos
        value_ms = float(match.group(2)) * 1000

        # Adiciona a métrica à nossa coleção de dados
        if metric_name not in collected_data:
            collected_data[metric_name] = []
        collected_data[metric_name].append(value_ms)

        # Imprime a nova métrica encontrada em tempo real
        print(f"  -> Métrica Capturada: {metric_name:<25} = {value_ms:>8.4f} ms")

def print_summary():
    """
    Imprime um resumo estatístico (média, min, max) dos dados coletados até agora.
    """
    print("\n--- RESUMO ESTATÍSTICO ATUAL ---")
    if not collected_data:
        print("Nenhum dado coletado ainda.")
        return

    # Imprime cabeçalho da tabela
    print(f"{'Métrica':<25} | {'Amostras':>8} | {'Média (ms)':>12} | {'Mín (ms)':>10} | {'Máx (ms)':>10}")
    print("-" * 75)

    for metric, values in collected_data.items():
        df = pd.Series(values)
        print(f"{metric:<25} | {len(df):>8} | {df.mean():>12.4f} | {df.min():>10.4f} | {df.max():>10.4f}")
    
    print("-" * 75)
    print("\n") # Espaço para as próximas métricas em tempo real

def tail_file():
    """
    Função principal que "segue" o final do arquivo de log, lendo novas linhas
    à medida que elas são escritas pela simulação.
    """
    try:
        # Garante que o arquivo exista antes de começar a observá-lo
        print("Aguardando a criação do arquivo de log pela simulação...")
        while not os.path.exists(LOG_FILE_PATH):
            time.sleep(1)
        print("Arquivo de log encontrado. Iniciando monitoramento.")

        with open(LOG_FILE_PATH, 'r') as file:
            # Pula para o final do arquivo
            file.seek(0, 2)
            summary_timer = time.time()

            while True:
                line = file.readline()
                if not line:
                    time.sleep(0.5)  # Espera por novas linhas se o arquivo estiver no fim
                    
                    # Imprime um resumo a cada 10 segundos se houver novos dados
                    if time.time() - summary_timer > 10 and collected_data:
                        print_summary()
                        summary_timer = time.time() # Reseta o timer
                    continue
                
                process_line(line)

    except FileNotFoundError:
        # Esta parte é um backup, mas o loop de espera acima deve evitar isso
        print(f"[ERRO] Arquivo de log '{LOG_FILE_PATH}' não foi encontrado.")
    except KeyboardInterrupt:
        # Ocorre quando o usuário pressiona Ctrl+C para parar o script
        print("\n\nMonitoramento encerrado pelo usuário.")
        # Imprime o resumo final com todos os dados coletados
        print_summary()

if __name__ == "__main__":
    tail_file()
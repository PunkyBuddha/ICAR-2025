import rosbag
import matplotlib.pyplot as plt
from std_msgs.msg import Float64 # Presumindo que os tópicos usam Float64. Mude se necessário.
from collections import defaultdict

# --- Configuração do Usuário ---

# Caminho para o seu arquivo rosbag
bag_file = '/home/fabricio/Flight_data/26-06 12:12.bag' # <-- MUDE AQUI

# Pares de tópicos que você deseja plotar juntos.
# Cada tupla na lista será um subplot.
topic_pairs = [
    ('/cf1/crazyflie/desiredRateIndi_p', '/cf1/crazyflie/error_phi'),
    ('/cf1/crazyflie/desiredRateIndi_q', '/cf1/crazyflie/error_theta'),
    ('/cf1/crazyflie/desiredRateIndi_r', '/cf1/crazyflie/error_psi')
]

# Campo da mensagem que você deseja extrair (ex: 'data' para std_msgs/Float64)
# Presumimos que todos os tópicos usam o mesmo nome de campo.
message_field = 'data'

# Título geral para a janela de plotagem
figure_title = 'Análise de Taxas Desejadas vs. Erros de Atitude'

# --- Fim da Configuração do Usuário ---

def read_rosbag_data(bag_file, all_topics, message_field):
    """
    Lê um arquivo rosbag e extrai dados de uma lista de tópicos.

    Args:
        bag_file (str): O caminho para o arquivo rosbag.
        all_topics (list): Uma lista com todos os nomes de tópicos a serem lidos.
        message_field (str): O campo da mensagem a ser extraído.

    Returns:
        dict: Um dicionário onde as chaves são os nomes dos tópicos e os valores
              são outros dicionários contendo 'timestamps' e 'values'.
    """
    # defaultdict simplifica a adição de novos dados
    data = defaultdict(lambda: {'timestamps': [], 'values': []})
    
    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            start_time = bag.get_start_time()
            # Itera sobre as mensagens, filtrando pela lista de todos os tópicos de interesse
            for topic, msg, t in bag.read_messages(topics=all_topics):
                # Extrai o timestamp relativo em segundos
                timestamp = t.to_sec() - start_time
                
                if hasattr(msg, message_field):
                    data[topic]['timestamps'].append(timestamp)
                    data[topic]['values'].append(getattr(msg, message_field))
                else:
                    # Imprime um aviso uma vez por tópico, se o campo não for encontrado
                    if not data[topic]['timestamps']: # Imprime só na primeira vez
                        print(f"Aviso: O campo '{message_field}' não foi encontrado na mensagem do tópico '{topic}'.")

    except FileNotFoundError:
        print(f"Erro: O arquivo rosbag '{bag_file}' não foi encontrado.")
        return None
    except Exception as e:
        print(f"Ocorreu um erro inesperado: {e}")
        return None
        
    return data

def plot_data_pairs(data, topic_pairs, figure_title):
    """
    Plota pares de dados em subplots.

    Args:
        data (dict): O dicionário de dados retornado por read_rosbag_data.
        topic_pairs (list): A lista de tuplas com os pares de tópicos.
        figure_title (str): O título da figura.
    """
    num_pairs = len(topic_pairs)
    # Cria uma figura e um conjunto de subplots (num_pairs linhas, 1 coluna)
    # sharex=True faz com que todos os gráficos compartilhem o mesmo eixo de tempo (eixo x)
    fig, axes = plt.subplots(num_pairs, 1, figsize=(14, 4 * num_pairs), sharex=True)
    fig.suptitle(figure_title, fontsize=16)

    # Se houver apenas um par, 'axes' não será um array, então o colocamos em uma lista
    if num_pairs == 1:
        axes = [axes]

    for i, (topic1, topic2) in enumerate(topic_pairs):
        ax = axes[i]
        
        # Plota os dados para o primeiro tópico do par
        if topic1 in data and data[topic1]['values']:
            ax.plot(data[topic1]['timestamps'], data[topic1]['values'], label=topic1.split('/')[-1])
        
        # Plota os dados para o segundo tópico do par
        if topic2 in data and data[topic2]['values']:
            ax.plot(data[topic2]['timestamps'], data[topic2]['values'], label=topic2.split('/')[-1], linestyle='--')

        ax.set_ylabel('Valor')
        ax.grid(True)
        ax.legend()
        ax.set_title(f'Comparação: {topic1.split("/")[-1]} & {topic2.split("/")[-1]}')

    # Adiciona o rótulo do eixo x apenas no último subplot (o de baixo)
    axes[-1].set_xlabel('Tempo (s)')

    # Ajusta o layout para evitar sobreposição de títulos
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == '__main__':
    # Cria uma lista única com todos os tópicos para ler do bag
    all_topics_to_read = [topic for pair in topic_pairs for topic in pair]
    
    # Lê os dados de todos os tópicos necessários
    bag_data = read_rosbag_data(bag_file, all_topics_to_read, message_field)
    
    # Se a leitura dos dados foi bem-sucedida, plota os gráficos
    if bag_data:
        plot_data_pairs(bag_data, topic_pairs, figure_title)
# from typing_extensions import Self
from operator import itemgetter
from queue import PriorityQueue
import time

from Grafos import Grafos


class GrafoFluxo(Grafos):

    def __init__(self, direcionado=False, ponderado=False):
        super().__init__(direcionado,
                         ponderado)  # Direcionado = False significa que é direcionado, e True que não é Direcionado
        self.lista = list()
        self.labels = dict()  # Contém os nomes de vértices

        self.vertices = 0
        self.arestas = 0

        # Fluxo
        self.fluxoMax = 0
        self.grafo_residual = list()

    def __str__(self):
        return str(self.lista)

    def openFile(self, filename):
        if self.vertices != 0 or self.arestas != 0:
            raise Exception('A instância do objeto GrafoLista não está vazia.')

        with open(filename, 'r') as arquivo:
            verticesArquivo, arestasArquivo, direcionadoArquivo, ponderadoArquivo = (arquivo.readline()).split()
            # Checa se é direcionado
            if int(direcionadoArquivo) == 1:
                self.direcionado = False
            else:
                self.direcionado = True
            # Checa se é ponderado
            if int(ponderadoArquivo) == 1:
                self.ponderado = True
            else:
                self.ponderado = False

            for vertice in range(int(verticesArquivo)):
                self.inserirVertice(str(vertice))

            try:

                if self.ponderado:
                    for aresta in range(int(arestasArquivo)):
                        origem, destino, peso = (arquivo.readline()).split()
                        self.inserirAresta(origem, destino, float(peso))
                else:
                    for aresta in range(int(arestasArquivo)):
                        origem, destino = (arquivo.readline()).split()
                        self.inserirAresta(origem, destino)

            except Exception:
                print('Linha vazia, ou com erros. Número de vértices lidas:', len(self.lista))

    def buscaProfunda(self, origem, destino):
        caminho = []
        visitado = set()
        return self._buscaProfunda(origem, destino, caminho, visitado)

    def _buscaProfunda(self, origem, destino, caminho=[], visitado=set()):
        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)
        caminho.append(origem)
        visitado.add(origem)

        if origem == destino:
            return caminho
        for (vizinho, peso) in self.lista[origem]:
            if vizinho not in visitado:
                resultado = self._buscaProfunda(vizinho, destino, caminho, visitado)
                if resultado is not None:
                    return resultado
        # caminho.pop() #diferença entre o visitado e o caminho, sem o pop, ele é igual a ordem visitada
        return None

    def _ordernarArestasDoVertice(self):
        for verticeId in range(len(self.lista)):
            self.lista[verticeId].sort(key=itemgetter(0))

    def inserirVertice(self, label):
        # Adiciona um vértice sem nenhuma aresta associada a ele, pode parecer
        # igual em ambos os casos, mas não é.
        # Precisamos adicionar esse vértice no vetor de vértices e também alocar
        # seu espaço para as arestas.

        # Define nome da vértice
        self.labels[label] = self.vertices
        self.vertices += 1

        self.lista.append(list())

    def removerVertice(self, label):
        # Remove um vértice do grafo, elimina a linha e coluna dele da matriz
        # e a referência dele da lista, junto com todas as arestas que chegam
        # e saem dele.
        # Converte o indice para label
        if isinstance(label, int):
            label = self.labelVertice(label)

        # Removendo o label
        idVertice = self.labels.get(label)
        self.labels.pop(label)
        # Balanceando os labels
        labelsTmp = dict()
        count = 0
        for label in self.labels.items():
            labelsTmp[label[0]] = count
            count += 1
        self.labels = labelsTmp
        # Diminuindo o número de vértices
        self.vertices -= 1

        listaTmp = list()
        for vertice in range(self.vertices + 1):
            if vertice == idVertice:
                continue
            listaTmp.append(self.lista[vertice])

        self.lista = listaTmp

    def labelVertice(self, indice):
        # Funções básicas para retornar o nome de um vértice.
        label = [k for k, v in self.labels.items() if v == indice]
        return (label[0])

    def imprimeGrafo(self, fluxo=True):
        # Imprimir o grafo no console, tentem deixar próximo da representação
        # dos slides (não precisa da grade da matriz).
        if fluxo and self.fluxoMax > 0:
            print("Fluxo Máximo:", self.fluxoMax)
            for valor in range(self.vertices):
                print(valor, ":", self.lista[valor])
        else:
            for valor in range(self.vertices):
                print(valor, ":", self.lista[valor])

    def inserirAresta(self, origem, destino, peso=1):
        # Essa operação deve ter um cuidado especial,
        # ela deve ser executada levando em conta o tipo do grafo.
        # No caso de um grafo ponderado o peso deve ser aplicado e
        # no caso de um grafo direcionado, uma ligação de volta deve ser adicionada;

        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)
        # Se não ponderado, o peso é igual a 1
        if not self.ponderado:
            peso = 1

        self.lista[origem].append((destino, peso))

        if self.direcionado and origem != destino:
            self.lista[destino].append((origem, peso))

        self._ordernarArestasDoVertice

        self.arestas += 1

    def removerAresta(self, origem, destino):
        # Remove uma aresta entre dois vértices no grafo, lembrando que no grafo
        # não direcionado deve ser removida a aresta de retorno também;
        # try:
        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)

        for idx, aresta in enumerate(self.lista[origem]):
            if destino in aresta:
                self.lista[origem].pop(idx)

        # self.lista[origem].remove(self.lista[origem][destino])

        # No direcionado, remove o do destino, a origem
        if self.direcionado:
            for idx, aresta in enumerate(self.lista[destino]):
                if origem in aresta:
                    self.lista[destino].pop(idx)
            # self.lista[destino].remove(self.lista[origem][destino])

        self._ordernarArestasDoVertice

        self.arestas -= 1

    # except:
    #   print('Aresta inválida.')

    def existeAresta(self, origem, destino):
        # Verifica a existência de uma aresta, aqui vemos uma diferença
        # grande entre matriz e lista.
        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)

        # Verifica se existe o destino na lista, ignorando o peso
        for aresta in self.lista[origem]:
            if destino in aresta:
                return True

        return False

    def pesoAresta(self, origem, destino):
        # Retorne o peso de uma aresta, aqui vemos uma diferença grande
        # entre matriz e lista.
        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)
        # print('\nPESOARESTA CHAMADO', end=' ')
        # print('origem:', origem, end=' ')
        # print('destino:', destino)

        # print('self.lista[origem]', self.lista[origem])
        # print('self.lista[origem][destino]', self.lista[origem][destino])
        # print('self.lista[origem][destino][1]', self.lista[origem][destino][1])
        return self.lista[origem][destino][1]

    def pesoArestaLabel(self, origem, destino):
        # Retorne o peso de uma aresta, aqui vemos uma diferença grande
        # entre matriz e lista.
        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)
        # print('\nPESOARESTA CHAMADO', end=' ')
        # print('origem:', origem, end=' ')
        # print('destino:', destino)

        # print('self.lista[origem]', self.lista[origem])
        # print('self.lista[origem][destino]', self.lista[origem][destino])
        # print('self.lista[origem][destino][1]', self.lista[origem][destino][1])
        # Verifica se existe o destino na lista, ignorando o peso
        for aresta in self.lista[origem]:
            # print('aresta:', aresta)
            # print('aresta:', aresta)
            # print('aresta[0]:', aresta[0])
            # print('destino:', aresta[0])
            if destino == aresta[0]:
                return aresta[1]
        return 0

    def updatePesoArestaLabel(self, origem, destino, peso):
        self.removerAresta(origem, destino)
        self.inserirAresta(origem, destino, peso)

    def retornarVizinhos(self, vertice):
        # Converte para numero
        if isinstance(vertice, str):
            vertice = self.labels.get(vertice)
        # Função para retorno dos vizinhos de um vértice, necessária pois não
        # teremos acesso a estrutura das arestas para os próximos algoritmos.
        vizinhos = list()

        # Retorna cada tupla com os vizinhos
        for aresta in self.lista[vertice]:
            tuplaTemp = (self.labelVertice(aresta[0]), aresta[0])
            vizinhos.append(tuplaTemp)

        return vizinhos

    def grauVertice(self, vertice):
        # Converte os labels para indices
        if isinstance(vertice, str):
            vertice = self.labels.get(vertice)
        if isinstance(vertice, list):  # Se já for uma lista, retorna o len dela
            return len(vertice)

        # print(vertice)
        # print(type(vertice))

        return len(self.lista[vertice])

    def labelsOrdenadosPorGrau(self):
        return sorted(self.labels, reverse=True, key=self.grauVertice)

    # Using BFS as a searching algorithm
    def busca_largura_fluxo(self, origem, destino, parent):
        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)
        visitado = [False] * (self.vertices)
        fila = [origem]
        visitado[origem] = True
        while fila:
            vertice_atual = fila.pop(0)
            for idx, aresta in enumerate(self.lista[vertice_atual]):
                if visitado[idx] is False and aresta[1] > 0:
                    fila.append(idx)
                    visitado[idx] = True
                    parent[idx] = vertice_atual

        return True if visitado[destino] else False

    # Algoritmo de BFS
    def busca_largura_fluxo_old(self, origem, destino, parent):
        # Converte os labels para indices
        if isinstance(origem, str):
            origem = self.labels.get(origem)
        if isinstance(destino, str):
            destino = self.labels.get(destino)
        visitados = [origem]
        visited = [False] * (self.vertices)
        fila = [origem]

        while fila:
            verticeAtual = fila.pop(0)
            vizinhos = self.retornarVizinhos(verticeAtual)
            for verticeVizinha in vizinhos:
            # for ind, val in enumerate(self.graph[u]):
                idxVerticeVizinha = self.labels.get(verticeVizinha[0])
                if idxVerticeVizinha not in visitados:
                    fila.append(idxVerticeVizinha)
                    visitados.append(idxVerticeVizinha)
                    parent[idxVerticeVizinha] = verticeAtual

        return True if destino in visitados else False

    # Applying fordfulkerson algorithm
    def ford_fulkerson(self, fonte, sorvedor, debug=True):
        # Converte os labels para indices
        if isinstance(fonte, str):
            fonte = self.labels.get(fonte)
        if isinstance(sorvedor, str):
            sorvedor = self.labels.get(sorvedor)

        parent = [-1] * self.vertices
        grafo_backup = self.lista.copy()

        if debug:
            print('--------------', 'START', sep='\n')
            print('Parent:', parent)
            count = 0

        while self.busca_largura_fluxo(fonte, sorvedor, parent):
            if debug:
                count += 1
                print('--------------', end='\n')
                print('Loop:', count)
                print('Parent:', parent)
            path_flow = float("Inf")
            vertice_atual = sorvedor

            if debug:
                # print('path_flow', path_flow)
                print('vertice_atual', vertice_atual)

            while vertice_atual != fonte:
                if debug:
                    print('vertice_atual != fonte', '---', sep='\n')
                    print('parent[s]', parent[vertice_atual])
                    # print('self.lista[parent[s]]', self.lista[parent[vertice_atual]])
                path_flow = min(path_flow, self.pesoArestaLabel(parent[vertice_atual], vertice_atual))
                vertice_atual = parent[vertice_atual]
                if debug:
                    print('path_flow', path_flow)

            # Adding the path flows
            self.fluxoMax += path_flow

            # Updating the residual values of edges
            vertice_residual = sorvedor
            while(vertice_residual != fonte):
                anterior_residual = parent[vertice_residual]

                peso_menos = self.pesoArestaLabel(
                    anterior_residual, vertice_residual) - path_flow
                self.updatePesoArestaLabel(
                    anterior_residual, vertice_residual, peso_menos)

                peso_mais = self.pesoArestaLabel(
                    vertice_residual, anterior_residual) + path_flow
                self.updatePesoArestaLabel(
                    vertice_residual, anterior_residual, peso_mais)

                vertice_residual = parent[vertice_residual]


            if count >= 10:
                break

        self.grafo_residual = self.lista.copy()
        self.lista = grafo_backup
        return self.fluxoMax


# Projeto MSX PicoVerse 2040

|PicoVerse Frente|PicoVerse Verso|
|---|---|
|![PicoVerse Frente](/images/2025-12-02_20-05.png)|![PicoVerse Verso](/images/2025-12-02_20-06.png)|

O PicoVerse 2040 é um cartucho para MSX baseado no Raspberry Pi Pico que utiliza firmware substituível para estender as capacidades do computador. Ao carregar diferentes imagens de firmware, o cartucho pode executar jogos e aplicações MSX e emular dispositivos de hardware adicionais (como mappers de ROM, RAM extra ou interfaces de armazenamento), adicionando efetivamente periféricos virtuais ao MSX. Um desses firmwares é o sistema MultiROM, que fornece um menu na tela para navegar e executar múltiplos títulos ROM armazenados no cartucho.

O cartucho também pode expor a porta USB‑C do Pico como um dispositivo de armazenamento em massa, permitindo copiar ROMs, DSKs e outros arquivos diretamente de um PC com Windows ou Linux para o cartucho.

Além disso, é possível usar uma variante de firmware Nextor com suporte a mapper de memória +240 KB para tirar proveito total do hardware do PicoVerse 2040 em sistemas MSX com memória limitada.

Estas são as funcionalidades disponíveis na versão atual do cartucho PicoVerse 2040:

- Sistema de menu MultiROM para seleção e execução de ROMs MSX.
- Suporte ao Nextor OS com opção de mapper de memória +240 KB.
- Suporte a dispositivo de armazenamento via USB para carregar ROMs e DSKs.
- Suporte a vários mappers de ROM MSX (PL-16, PL-32, KonSCC, Linear, ASC-08, ASC-16, Konami, NEO-8, NEO-16).
- Compatibilidade com sistemas MSX, MSX2 e MSX2+.

## Manual do MultiROM UF2 Creator

Esta seção documenta a ferramenta de console `multirom` usada para gerar imagens UF2 (`multirom.uf2`) que gravam o cartucho PicoVerse 2040.

Por padrão, a ferramenta `multirom` varre arquivos ROM MSX em um diretório e os empacota em uma única imagem UF2 que pode ser gravada no Raspberry Pi Pico. A imagem resultante normalmente contém o binário do firmware do Pico, o ROM de menu do MultiROM, uma área de configuração descrevendo cada entrada de ROM e os próprios payloads das ROMs.

> **Observação:** Um máximo de 128 ROMs pode ser incluído em uma única imagem, com limite de tamanho total aproximado de 16 MB.

Dependendo das opções fornecidas, `multirom` também pode gerar imagens UF2 que inicializam diretamente em um firmware personalizado em vez do menu MultiROM. Por exemplo, opções permitem produzir UF2 contendo um firmware que implementa o Nextor OS. Nesse modo, a porta USB‑C do Pico pode ser usada como dispositivo de armazenamento em massa para carregar ROMs, DSKs e outros arquivos a partir do Nextor (por exemplo, via SofaRun).

## Visão geral

Quando executado sem opções, o executável `multirom.exe` varre o diretório de trabalho atual procurando arquivos ROM MSX (`.ROM` ou `.rom`), analisa cada ROM para tentar identificar o tipo de mapper, monta uma tabela de configuração descrevendo cada ROM (nome, byte do mapper, tamanho, offset) e incorpora essa tabela em um pedaço do ROM de menu MSX. A ferramenta então concatena o blob do firmware do Pico, o slice do menu, a área de configuração e os payloads das ROMs e serializa toda a imagem em um arquivo UF2 chamado `multirom.uf2`.

![alt text](/images/2025-11-29_20-49.png)

O arquivo UF2 (geralmente multirom.uf2) pode então ser copiado para o dispositivo de armazenamento em massa do Pico para gravar a imagem combinada. É necessário conectar o Pico pressionando o botão BOOTSEL para entrar no modo de gravação UF2. Em seguida, um novo drive USB chamado `RPI-RP2` aparece e você pode copiar `multirom.uf2` para ele. Após a cópia, desconecte o Pico e insira o cartucho no MSX para inicializar o menu MultiROM.

Os nomes dos arquivos ROM são usados como nomes das entradas no menu MSX. Há um limite de 50 caracteres por nome. Um efeito de rolagem é usado para mostrar nomes mais longos no menu MSX, mas se o nome exceder 50 caracteres ele será truncado.

![alt text](/images/multirom_2040_menu.png)

Se você deseja usar o Nextor com seu cartucho PicoVerse 2040, é preciso executar a ferramenta com a opção `-s1` ou `-s2` para incluir o ROM Nextor incorporado na imagem. A opção `-s1` inclui o ROM Nextor padrão sem suporte ao mapper de memória, enquanto `-s2` inclui um ROM Nextor com suporte a mapper +240 KB. O ROM Nextor incorporado será o firmware único carregado na inicialização e o menu MultiROM não ficará disponível. Você poderá então usar o SofaRun para carregar ROMs e DSKs a partir do armazenamento em massa do Pico.

> **Observação:** Para usar um pendrive USB você pode precisar de um adaptador ou cabo OTG. Isso permite converter a porta USB‑C para uma porta USB‑A fêmea padrão.

> **Observação:** A opção `-n` inclui um ROM Nextor experimental incorporado que funciona em modelos MSX2 específicos. Esta versão usa um método de comunicação diferente (baseado em portas de I/O) e pode não funcionar em todos os sistemas.

## Uso via linha de comando

Somente executáveis para Microsoft Windows são fornecidos por enquanto (`multirom.exe`).

### Uso básico:

```
multirom.exe [options]
```

### Opções:
- `-n`, `--nextor` : Inclui o ROM NEXTOR beta incorporado (conforme configurado) na saída. Esta opção ainda é experimental e atualmente funciona apenas em modelos MSX2 específicos.
- `-h`, `--help`   : Exibe a ajuda de uso e sai.
- `-o <filename>`, `--output <filename>` : Define o nome do arquivo UF2 de saída (padrão `multirom.uf2`).
- `-s1`            : Cartucho inicializa com Sunrise IDE Nextor (sem mapper de memória).
- `-s2`            : Cartucho inicializa com Sunrise IDE Nextor (+240 KB de mapper de memória).
- Se nenhuma das opções `-s1` ou `-s2` for especificada, a ferramenta produz uma imagem MultiROM com o menu.
- Se for necessário forçar um tipo de mapper específico para um arquivo ROM, você pode anexar uma tag de mapper antes da extensão `.ROM` no nome do arquivo. A tag não diferencia maiúsculas de minúsculas. Por exemplo, nomear um arquivo `Knight Mare.PL-32.ROM` força o uso do mapper PL-32 para essa ROM. Tags como `SYSTEM` são ignoradas. A lista de tags possíveis é: `PL-16,  PL-32,  KonSCC,  Linear,  ASC-08,  ASC-16,  Konami,  NEO-8,  NEO-16`

### Exemplos
- Produz o arquivo multirom.uf2 com o menu MultiROM e todas as `.ROM` do diretório atual. Você pode executar a ferramenta pelo prompt de comando ou dando um duplo clique no executável:
  ```
  multirom.exe
  ```

- Produz o arquivo multirom.uf2 com o ROM Sunrise IDE Nextor (sem mapper de memória):
  ```
  multirom.exe -s1
  ```
- Produz o arquivo multirom.uf2 com o ROM Sunrise IDE Nextor (+240 KB de mapper de memória):
  ```
  multirom.exe -s2
  ```

## Como funciona (visão geral)

1. A ferramenta varre o diretório de trabalho atual em busca de arquivos terminados em `.ROM` ou `.rom`. Para cada arquivo:
   - Extrai um nome de exibição (nome do arquivo sem extensão, truncado a 50 caracteres).
   - Obtém o tamanho do arquivo e valida se está entre `MIN_ROM_SIZE` e `MAX_ROM_SIZE`.
   - Chama `detect_rom_type()` para heuristically determinar o byte do mapper a ser usado na entrada de configuração. Se uma tag de mapper estiver presente no nome do arquivo, ela substitui a detecção.
   - Se a detecção de mapper falhar, o arquivo é ignorado.
   - Serializa o registro de configuração por ROM (nome de 50 bytes, 1 byte de mapper, 4 bytes de tamanho LE, 4 bytes de offset no flash LE) na área de configuração.
2. Após a varredura, a ferramenta concatena (na ordem): binário do firmware do Pico incorporado, um slice inicial do ROM de menu MSX (`MENU_COPY_SIZE` bytes), a área de configuração completa (`CONFIG_AREA_SIZE` bytes), o ROM NEXTOR opcional e, em seguida, os payloads das ROMs descobertas na ordem de descoberta.
3. O payload combinado é escrito como um arquivo UF2 chamado `multirom.uf2` usando `create_uf2_file()`, que produz blocos UF2 de 256 bytes com payload direcionados ao endereço de flash do Pico `0x10000000`.

## Heurísticas de detecção de mappers
- `detect_rom_type()` implementa uma combinação de verificações de assinatura (cabeçalho "AB", tags `ROM_NEO8` / `ROM_NE16`) e varredura heurística de opcodes e endereços para identificar mappers MSX comuns, incluindo (mas não limitado a):
  - Plain 16KB (byte do mapper 1) — verificação de cabeçalho AB em 16KB
  - Plain 32KB (byte do mapper 2) — verificação de cabeçalho AB em 32KB
  - Linear0 mapper (byte do mapper 4) — verificação de layout AB especial
  - NEO8 (byte do mapper 8) e NEO16 (byte do mapper 9)
  - Konami, Konami SCC, ASCII8, ASCII16 e outros via pontuação ponderada
- Se nenhum mapper puder ser detectado com confiabilidade, a ferramenta ignora a ROM e reporta "unsupported mapper". Lembre-se que você pode forçar um mapper via tag no nome do arquivo. As tags não diferenciam maiúsculas/minúsculas e estão listadas acima.
- Apenas os seguintes mappers são suportados na área de configuração e no menu: `PL-16,  PL-32,  KonSCC,  Linear,  ASC-08,  ASC-16,  Konami,  NEO-8,  NEO-16`

## Usando o menu Seletor de ROMs do MSX

Ao ligar o MSX com o cartucho PicoVerse 2040 inserido, o menu MultiROM aparece, mostrando a lista de ROMs disponíveis. Você pode navegar pelo menu usando as teclas de seta do teclado.

![alt text](/images/multirom_2040_menu.png)

Use as teclas de seta para cima e para baixo para mover o cursor de seleção pela lista de ROMs. Se você tiver mais de 19 ROMs, use as teclas laterais (Esquerda e Direita) para percorrer páginas de entradas.

Pressione Enter ou Espaço para executar a ROM selecionada. O MSX tentará inicializar a ROM usando as configurações apropriadas do mapper.

A qualquer momento no menu, você pode pressionar H para ver a tela de ajuda com instruções básicas. Pressione qualquer tecla para voltar ao menu principal.

## Usando Nextor com o cartucho PicoVerse 2040

Se você gravou o cartucho com um firmware Nextor (usando as opções `-s1` ou `-s2`), o cartucho inicializará diretamente no Nextor em vez do menu MultiROM. Essas opções comunicam-se com o MSX via I/O mapeado em memória, exigindo que o cartucho seja inserido em um slot de cartucho primário.

Com o Nextor em execução, você pode usar o SofaRun ou qualquer outro lançador compatível com Nextor para carregar ROMs e DSKs a partir do dispositivo de armazenamento em massa do Pico. Pode ser necessário um adaptador ou cabo OTG para conectar pendrives USB‑A padrão à porta USB‑C do cartucho.

Em sistemas MSX2 ou MSX2+, com a versão Nextor de +240 KB de memória (-s2), o cartucho adicionará a memória extra ao sistema e a sequência de BIOS refletirá o aumento de RAM. Alguns modelos de MSX exibem a memória expandida total durante a inicialização.

A opção -n da ferramenta multirom inclui um ROM Nextor experimental incorporado que pode ser usado em modelos MSX2 específicos. Entretanto, essa opção ainda está em desenvolvimento e pode não funcionar em todos os sistemas. Esta versão usa um método diferente (baseado em portas de I/O) para comunicar-se com o MSX, permitindo seu uso em sistemas onde o slot do cartucho não é primário, por exemplo com expansores de slot.

Mais detalhes sobre o protocolo usado para comunicar-se com o computador MSX podem ser encontrados no documento Nextor-Pico-Bridge-Protocol.md. Os detalhes da comunicação com o firmware Sunrise IDE Nextor estão no documento Sunrise-Nextor.md.

### Como preparar um pendrive para o Nextor

Para preparar um pendrive USB para uso com o Nextor no cartucho PicoVerse 2040, siga estes passos:

1. Conecte o pendrive USB ao seu PC.
2. Crie uma partição FAT16 com no máximo 4 GB no pendrive. Você pode usar a ferramenta FDISK do Nextor (CALL FDISK enquanto estiver no MSX Basic) ou software de particionamento de terceiros.
3. Copie os arquivos do sistema Nextor para o diretório raiz da partição FAT16. Você pode obter os arquivos do sistema Nextor na distribuição oficial ou repositório. Também é necessário o arquivo COMMAND2 para o shell de comandos:
   1.  [Página de downloads do Nextor](https://github.com/Konamiman/Nextor/releases)
   2.  [Pagina de download do Command2](http://www.tni.nl/products/command2.html)
4. Copie quaisquer ROMs MSX (`.ROM`) ou imagens de disco (`.DSK`) que deseja usar com o Nextor para o diretório raiz do pendrive.
5. Instale o SofaRun ou outro lançador compatível com Nextor no pendrive se pretende usá-lo para iniciar ROMs e DSKs. Você pode baixar o SofaRun em: [SofaRun](https://www.louthrax.net/mgr/sofarun.html)
6. Ejete o pendrive com segurança do seu PC.
7. Conecte o pendrive ao cartucho PicoVerse 2040 usando um adaptador ou cabo OTG, se necessário.

> **Observação:** Nem todos os pendrives USB são compatíveis com o cartucho PicoVerse 2040. Se encontrar problemas, tente usar outra marca ou modelo.
>
> **Observação:** Lembre-se que o Nextor precisa de no mínimo 128 KB de RAM para operar. Se você estiver usando o firmware Nextor padrão (opção -s1), certifique-se de que seu MSX tenha RAM suficiente disponível.

## Ideias de melhorias
- Melhorar as heurísticas de detecção do tipo de ROM para cobrir mais mappers e casos extremos.
- Implementar tela de configuração para cada entrada de ROM (nome, override de mapper, etc.).
- Adicionar suporte a mais mappers de ROM.
- Implementar um menu gráfico com melhor navegação e exibição de informações da ROM.
- Adicionar suporte para salvar/carregar configuração do menu para preservar preferências do usuário.
- Suportar também arquivos DSK, com entradas de configuração adequadas.
- Adicionar suporte a temas personalizados para o menu.
- Permitir baixar ROMs de URLs e incorporá-las diretamente.
- Permitir o uso do joystick para navegar no menu.

## Problemas conhecidos

- A inclusão do ROM Nextor incorporado ainda é experimental e pode não funcionar em todos os modelos MSX2.
- Algumas ROMs com mappers pouco comuns podem não ser detectadas corretamente e serão ignoradas, a menos que uma tag de mapper válida seja usada para forçar a detecção.
- A ferramenta MultiROM atualmente suporta apenas Windows. Versões para Linux e macOS não estão disponíveis ainda.
- A ferramenta não valida atualmente a integridade dos arquivos ROM além de tamanho e verificações básicas de cabeçalho. ROMs corrompidas podem causar comportamento inesperado.
- Devido à natureza da ferramenta MultiROM (incorporação de múltiplos arquivos em um único UF2), alguns antivírus podem sinalizar o executável como suspeito. Isso é um falso positivo; certifique-se de baixar a ferramenta de uma fonte confiável.
- O menu MultiROM não suporta arquivos DSK; apenas arquivos ROM são listados e executados.
- A ferramenta não suporta atualmente subdiretórios; apenas ROMs no diretório de trabalho atual são processadas.
- A memória flash do Pico pode se desgastar após muitos ciclos de gravação. Evite regravações excessivas do cartucho.

## Modelos MSX testados

O cartucho PicoVerse 2040 com firmware MultiROM foi testado nos seguintes modelos MSX:

| Model | Tipo | Status | Comentários |
| --- | --- | --- | --- |
| Adermir Carchano Expert 4 | MSX2+ | OK | Operação verificada |
| Gradiente Expert | MSX1 | OK | Operação verificada |
| JFF MSX | MSX1 | OK | Operação verificada |
| MSX Book | MSX2+ (clone FPGA) | OK | Operação verificada |
| MSX One | MSX1 | Not OK | Cartucho não reconhecido |
| National FS-4500 | MSX1 | OK | Operação verificada |
| Omega MSX | MSX2+ | OK | Operação verificada |
| Panasonic FS-A1GT | TurboR | OK | Operação verificada |
| Panasonic FS-A1ST | TurboR | OK | Operação verificada |
| Panasonic FS-A1WX | MSX2+ | OK | Operação verificada |
| Panasonic FS-A1WSX | MSX2+ | OK | Operação verificada |
| Sharp HotBit HB8000 | MSX1 | OK | Operação verificada |
| SMX-HB | MSX2+ (clone FPGA) | OK | Operação verificada |
| Sony HB-F1XD | MSX2 | OK | Operação verificada |
| Sony HB-F1XDJ | MSX2 | OK | Operação verificada |
| Sony HB-F1XV | MSX2+ | OK | Operação verificada |
| TRHMSX | MSX2+ (clone FPGA) | OK | Operação verificada |
| uMSX | MSX2+ (clone FPGA) | OK | Operação verificada |
| Yamaha YIS604 | MSX1 | OK | Operação verificada |

O cartucho PicoVerse 2040 com o firmware Sunrise Nextor +240K (opção `-s2`) foi testado nos seguintes modelos MSX:

| Model | Tipo | Status | Comentários |
| --- | --- | --- | --- |
| Gradiente Expert | MSX1 | Not OK | Mapper de memória não funcionou, Nextor não funcional |
| Sharp HotBit HB8000 (MSX1) | MSX1| Not OK | Mapper de memória não funcionou, Nextor não funcional |
| TRHMSX (MSX2+, clone FPGA) | MSX2+ | OK | Operação verificada |
| uMSX (MSX2+, clone FPGA) | MSX2+ | OK | Operação verificada |

A ROM experimental do Nextor incorporada (opção `-n`) foi testada nos seguintes modelos:

| Modelo | Tipo | Status | Comentários |
| --- | --- | --- | --- |
| Omega MSX | MSX2+ | Não OK | USB não detectado |
| Sony HB-F1XD | MSX2 | Não OK | Erro ao ler setor do disco |
| TRHMSX | MSX2+ (clone FPGA) | OK | Operação verificada |
| uMSX | MSX2+ (clone FPGA) | OK | Operação verificada |

Author: Cristiano Almeida Goncalves
Last updated: 12/25/2025
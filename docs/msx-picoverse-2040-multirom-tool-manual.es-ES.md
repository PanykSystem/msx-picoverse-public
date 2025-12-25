# Proyecto MSX PicoVerse 2040

|PicoVerse Anverso|PicoVerse Reverso|
|---|---|
|![PicoVerse Front](/images/2025-12-02_20-05.png)|![PicoVerse Back](/images/2025-12-02_20-06.png)|

El PicoVerse 2040 es un cartucho para MSX basado en Raspberry Pi Pico que utiliza firmware intercambiable para ampliar las capacidades del ordenador. Cargando distintas imágenes de firmware, el cartucho puede ejecutar juegos y aplicaciones MSX y emular dispositivos hardware adicionales (como mapeadores de ROM, RAM extra o interfaces de almacenamiento), añadiendo efectivamente periféricos virtuales al MSX. Uno de estos firmwares es el sistema MultiROM, que proporciona un menú en pantalla para explorar y lanzar múltiples títulos ROM almacenados en el cartucho.

El cartucho también puede exponer el puerto USB‑C del Pico como un dispositivo de almacenamiento masivo, permitiendo copiar ROMs, DSKs y otros archivos directamente desde un PC con Windows o Linux al cartucho.

Además, se puede usar una variante de firmware Nextor con soporte de mapeador de memoria +240 KB para aprovechar al máximo el hardware del PicoVerse 2040 en sistemas MSX con memoria limitada.

Estas son las características disponibles en la versión actual del cartucho PicoVerse 2040:

* Sistema de menú MultiROM para seleccionar y lanzar ROMs MSX.
* Soporte de Nextor OS con mapeador de memoria opcional de +240 KB.
* Soporte de dispositivo de almacenamiento USB para cargar ROMs y DSKs.
* Soporte para varios mapeadores de ROM MSX (PL-16, PL-32, KonSCC, Linear, ASC-08, ASC-16, Konami, NEO-8, NEO-16).
* Compatibilidad con sistemas MSX, MSX2 y MSX2+.

## Manual del creador de UF2 MultiROM

Esta sección documenta la herramienta de consola `multirom` usada para generar imágenes UF2 (`multirom.uf2`) que programan el cartucho PicoVerse 2040.

Por defecto, la herramienta `multirom` escanea archivos ROM MSX en un directorio y los empaqueta en una única imagen UF2 que puede ser flasheada en la Raspberry Pi Pico. La imagen resultante típicamente contiene el binario del firmware del Pico, la ROM del menú MSX de MultiROM, un área de configuración que describe cada entrada ROM y las propias cargas útiles de las ROM.

> **Nota:** Se pueden incluir hasta 128 ROMs en una sola imagen, con un límite de tamaño total aproximado de 16 MB.

Dependiendo de las opciones proporcionadas, `multirom` también puede producir imágenes UF2 que arranquen directamente en un firmware personalizado en lugar del menú MultiROM. Por ejemplo, se pueden generar UF2 con un firmware que implemente Nextor. En ese modo, el puerto USB‑C del Pico puede usarse como dispositivo de almacenamiento masivo para cargar ROMs, DSKs y otros archivos desde Nextor (por ejemplo, usando SofaRun).

## Visión general

Si se ejecuta sin opciones, la herramienta `multirom.exe` escanea el directorio de trabajo actual en busca de archivos ROM MSX (`.ROM` o `.rom`), analiza cada ROM para adivinar el tipo de mapeador, construye una tabla de configuración que describe cada ROM (nombre, byte de mapeador, tamaño, offset) e inserta esta tabla en una porción de la ROM del menú MSX. La herramienta concatena el blob del firmware del Pico, la porción del menú, el área de configuración y las cargas de las ROMs y serializa toda la imagen en un archivo UF2 llamado `multirom.uf2`.

![alt text](/images/2025-11-29_20-49.png)

El archivo UF2 (normalmente multirom.uf2) puede copiarse al dispositivo de almacenamiento masivo USB del Pico para flashear la imagen combinada. Debe conectar el Pico mientras mantiene presionado el botón BOOTSEL para entrar en el modo de flasheo UF2. Entonces aparece una nueva unidad USB llamada `RPI-RP2`, y puede copiar `multirom.uf2` en ella. Tras completar la copia, desconecte el Pico e inserte el cartucho en su MSX para arrancar el menú MultiROM.

Los nombres de los archivos ROM se usan para nombrar las entradas en el menú MSX. Hay un límite de 50 caracteres por nombre. Se utiliza un efecto de desplazamiento para mostrar nombres más largos en el menú MSX, pero si el nombre supera los 50 caracteres será truncado.

![alt text](/images/multirom_2040_menu.png)

Si desea usar Nextor con su cartucho PicoVerse 2040 debe ejecutar la herramienta con la opción `-s1` o `-s2` para incluir la ROM Nextor embebida en la imagen. La opción `-s1` incluye la ROM Nextor estándar sin soporte de mapeador de memoria, mientras que `-s2` incluye una ROM Nextor con soporte de mapeador de +240 KB. La ROM Nextor embebida será el único firmware cargado al arrancar y el menú MultiROM no estará disponible. Entonces podrá usar SofaRun para cargar ROMs y DSKs desde el almacenamiento masivo USB del Pico.

> **Nota:** Para usar una memoria USB puede necesitar un adaptador o cable OTG. Esto se puede usar para convertir el puerto USB‑C a un conector USB‑A hembra estándar.

> **Nota:** La opción `-n` incluye una ROM Nextor embebida experimental que funciona en modelos MSX2 específicos. Esta versión usa un método de comunicación distinto (basado en puertos IO) y puede no funcionar en todos los sistemas.

## Uso en línea de comandos

Solo se proporcionan ejecutables para Microsoft Windows por ahora (`multirom.exe`).

### Uso básico:

```
multirom.exe [options]
```

### Opciones:
- `-n`, `--nextor` : Incluye la ROM NEXTOR beta embebida desde la configuración y la salida. Esta opción aún es experimental y por el momento solo funciona en modelos MSX2 específicos.
- `-h`, `--help`   : Muestra la ayuda de uso y sale.
- `-o <filename>`, `--output <filename>` : Establece el nombre del archivo UF2 de salida (por defecto `multirom.uf2`).
- `-s1`            : El cartucho arranca con Sunrise IDE Nextor (sin mapeador de memoria).
- `-s2`            : El cartucho arranca con Sunrise IDE Nextor (+240 KB de mapeador de memoria).
- Si no se especifica `-s1` ni `-s2`, la herramienta produce una imagen MultiROM con el menú.
- Si necesita forzar un tipo de mapeador específico para un archivo ROM, puede añadir una etiqueta de mapeador antes de la extensión `.ROM` en el nombre del archivo. La etiqueta no distingue entre mayúsculas y minúsculas. Por ejemplo, nombrar un archivo `Knight Mare.PL-32.ROM` fuerza el uso del mapeador PL-32 para esa ROM. Etiquetas como `SYSTEM` se ignoran. La lista de etiquetas posibles es: `PL-16,  PL-32,  KonSCC,  Linear,  ASC-08,  ASC-16,  Konami,  NEO-8,  NEO-16`

### Ejemplos
- Produce el archivo multirom.uf2 con el menú MultiROM y todas las `.ROM` del directorio actual. Puede ejecutar la herramienta desde el símbolo del sistema o simplemente haciendo doble clic en el ejecutable:
  ```
  multirom.exe
  ```

- Produce el archivo multirom.uf2 con la ROM Sunrise IDE Nextor (sin mapeador de memoria):
  ```
  multirom.exe -s1
  ```
- Produce el archivo multirom.uf2 con la ROM Sunrise IDE Nextor (+240 KB de mapeador de memoria):
  ```
  multirom.exe -s2
  ```

## Cómo funciona (a alto nivel)

1. La herramienta escanea el directorio de trabajo actual en busca de archivos que terminen en `.ROM` o `.rom`. Para cada archivo:
   - Extrae un nombre para mostrar (nombre de archivo sin extensión, truncado a 50 caracteres).
   - Obtiene el tamaño del archivo y valida que esté entre `MIN_ROM_SIZE` y `MAX_ROM_SIZE`.
   - Llama a `detect_rom_type()` para determinar heurísticamente el byte de mapeador a usar en la entrada de configuración. Si hay una etiqueta de mapeador presente en el nombre del archivo, esta anula la detección.
   - Si la detección del mapeador falla, el archivo se omite.
   - Serializa el registro de configuración por ROM (nombre de 50 bytes, 1 byte de mapeador, 4 bytes de tamaño LE, 4 bytes de offset en flash LE) en el área de configuración.
2. Tras el escaneo, la herramienta concatena (en este orden): binario del firmware embebido del Pico, un fragmento inicial de la ROM del menú MSX (`MENU_COPY_SIZE` bytes), el área completa de configuración (`CONFIG_AREA_SIZE` bytes), la ROM NEXTOR opcional y luego las cargas útiles de las ROMs descubiertas en el orden de descubrimiento.
3. La carga combinada se escribe en un archivo UF2 llamado `multirom.uf2` usando `create_uf2_file()` que produce bloques UF2 de 256 bytes dirigidos a la dirección de flash del Pico `0x10000000`.

## Heurísticas de detección de mapeadores
- `detect_rom_type()` implementa una combinación de comprobaciones de firma (cabecera "AB", etiquetas `ROM_NEO8` / `ROM_NE16`) y un escaneo heurístico de opcodes y direcciones para elegir mapeadores MSX comunes, incluyendo (pero no limitado a):
  - Plain 16KB (byte de mapeador 1) — comprobación de cabecera AB de 16KB
  - Plain 32KB (byte de mapeador 2) — comprobación de cabecera AB de 32KB
  - Mapeador Linear0 (byte de mapeador 4) — comprobación de disposición AB especial
  - NEO8 (byte de mapeador 8) y NEO16 (byte de mapeador 9)
  - Konami, Konami SCC, ASCII8, ASCII16 y otros mediante puntuación ponderada
- Si no se puede detectar un mapeador de forma fiable, la herramienta omite la ROM y reporta "unsupported mapper". Recuerde que puede forzar un mapeador mediante la etiqueta en el nombre del archivo. Las etiquetas no distinguen mayúsculas/minúsculas y se listan más arriba.
- Solo los siguientes mapeadores están soportados en el área de configuración y el menú: `PL-16,  PL-32,  KonSCC,  Linear,  ASC-08,  ASC-16,  Konami,  NEO-8,  NEO-16`

## Uso del menú selector de ROMs MSX

Al encender el MSX con el cartucho PicoVerse 2040 insertado, aparece el menú MultiROM mostrando la lista de ROMs disponibles. Puede navegar por el menú usando las teclas de flecha del teclado.

![alt text](/images/multirom_2040_menu.png)

Use las teclas de flecha Arriba y Abajo para mover el cursor de selección por la lista de ROMs. Si tiene más de 19 ROMs, use las flechas laterales (Izquierda y Derecha) para desplazarse por las páginas de entradas.

Pulse Enter o Espacio para lanzar la ROM seleccionada. El MSX intentará arrancar la ROM usando los ajustes de mapeador apropiados.

En cualquier momento mientras esté en el menú, puede pulsar la tecla H para leer la pantalla de ayuda con instrucciones básicas. Pulse cualquier tecla para volver al menú principal.

## Uso de Nextor con el cartucho PicoVerse 2040

Si flasheó el cartucho con un firmware Nextor (usando las opciones `-s1` o `-s2`), el cartucho arrancará directamente en Nextor en lugar del menú MultiROM. Esas opciones se comunican con el MSX usando E/S mapeada en memoria, por lo que el cartucho debe estar insertado en una ranura de cartucho primaria.

Una vez que Nextor esté en ejecución, puede usar SofaRun o cualquier otro lanzador compatible con Nextor para cargar ROMs y DSKs desde el dispositivo de almacenamiento masivo USB del Pico. Puede necesitar un adaptador o cable OTG para conectar memorias USB‑A estándar al puerto USB‑C del cartucho.

En sistemas MSX2 o MSX2+, con la versión de Nextor con +240 KB de memoria (-s2), el cartucho añadirá la memoria extra al sistema y la secuencia del BIOS reflejará el aumento de RAM. Algunos modelos MSX muestran la memoria total expandida durante el arranque.

La opción -n de la herramienta multirom incluye una ROM Nextor embebida experimental que puede usarse en modelos MSX2 específicos. Sin embargo, esta opción sigue en desarrollo y puede no funcionar en todos los sistemas. Esta versión usa un método distinto (basado en puertos IO) para comunicarse con el MSX, permitiendo que funcione en sistemas donde la ranura de cartucho no es primaria, por ejemplo con expansores de ranura.

Más detalles sobre el protocolo usado para comunicarse con el ordenador MSX se pueden encontrar en el documento Nextor-Pico-Bridge-Protocol.md. Los detalles de la comunicación con el firmware Sunrise IDE Nextor se encuentran en el documento Sunrise-Nextor.md.

### Cómo preparar una memoria USB para Nextor

Para preparar una memoria USB para usar con Nextor en el cartucho PicoVerse 2040, siga estos pasos:

1. Conecte la memoria USB a su PC.
2. Cree una partición FAT16 de máximo 4 GB en la memoria USB. Puede usar la herramienta incorporada de Nextor (CALL FDISK desde MSX Basic) o software de particionado de terceros para esto.
3. Copie los archivos del sistema Nextor al directorio raíz de la partición FAT16. Puede obtener los archivos del sistema Nextor desde la distribución oficial o el repositorio. También necesita el archivo COMMAND2 para la shell de comandos:
   1.  [Nextor Download Page](https://github.com/Konamiman/Nextor/releases)
   2.  [Command2 Download Page](http://www.tni.nl/products/command2.html)
4. Copie las ROMs MSX (`.ROM`) o imágenes de disco (`.DSK`) que desee usar con Nextor al directorio raíz de la memoria USB.
5. Instale SofaRun u otro lanzador compatible con Nextor en la memoria USB si planea usarla para lanzar ROMs y DSKs. Puede descargar SofaRun desde su fuente oficial aquí: [SofaRun](https://www.louthrax.net/mgr/sofarun.html)
6. Expulse la memoria USB de forma segura desde su PC.
7. Conecte la memoria USB al cartucho PicoVerse 2040 usando un adaptador o cable OTG si es necesario.

> **Nota:** No todas las memorias USB son compatibles con el cartucho PicoVerse 2040. Si encuentra problemas, pruebe con otra marca o modelo.
>
> **Nota:** Recuerde que Nextor necesita un mínimo de 128 KB de RAM para operar. Si usa el firmware Nextor estándar (-s1), asegúrese de que su MSX dispone de RAM suficiente.

## Ideas de mejora
- Mejorar las heurísticas de detección de tipo de ROM para cubrir más mapeadores y casos límite.
- Implementar una pantalla de configuración para cada entrada ROM (nombre, anulación de mapeador, etc.).
- Añadir soporte para más mapeadores de ROM.
- Implementar un menú gráfico con mejor navegación e información de las ROMs.
- Añadir soporte para guardar/cargar la configuración del menú para preservar preferencias del usuario.
- Soportar archivos DSK también, con entradas de configuración adecuadas.
- Añadir soporte para temas personalizados del menú.
- Permitir la descarga de ROMs desde URLs de Internet e incrustarlas directamente.
- Permitir el uso del joystick para navegar el menú.

## Problemas conocidos

- La inclusión de la ROM Nextor embebida sigue siendo experimental y puede no funcionar en todos los modelos MSX2.
- Algunas ROMs con mapeadores poco comunes pueden no detectarse correctamente y serán omitidas a menos que se use una etiqueta de mapeador válida para forzar la detección.
- La herramienta MultiROM actualmente solo soporta Windows. No hay versiones para Linux ni macOS por ahora.
- La herramienta no valida actualmente la integridad de los archivos ROM más allá del tamaño y comprobaciones básicas de cabecera. ROMs corruptas pueden provocar comportamiento inesperado.
- Debido a la naturaleza de la herramienta MultiROM (incrustar múltiples archivos en un único UF2), algunos antivirus pueden marcar el ejecutable como sospechoso. Esto es un falso positivo; asegúrese de descargar la herramienta desde una fuente de confianza.
- El menú MultiROM no soporta archivos DSK; solo se listan y lanzan archivos ROM.
- La herramienta no soporta actualmente subdirectorios; solo se procesan ROMs en el directorio de trabajo actual.
- La memoria flash del Pico puede desgastarse tras muchos ciclos de escritura. Evite reflashear el cartucho en exceso.

## Modelos MSX probados

El cartucho PicoVerse 2040 con firmware MultiROM se ha probado en los siguientes modelos MSX:

| Modelo | Tipo | Estado | Comentarios |
| --- | --- | --- | --- |
| Adermir Carchano Expert 4 | MSX2+ | OK | Operación verificada |
| Gradiente Expert | MSX1 | OK | Operación verificada |
| JFF MSX | MSX1 | OK | Operación verificada |
| MSX Book | MSX2+ (clon FPGA) | OK | Operación verificada |
| MSX One | MSX1 | No OK | Cartucho no reconocido |
| National FS-4500 | MSX1 | OK | Operación verificada |
| Omega MSX | MSX2+ | OK | Operación verificada |
| Panasonic FS-A1GT | TurboR | OK | Operación verificada |
| Panasonic FS-A1ST | TurboR | OK | Operación verificada |
| Panasonic FS-A1WX | MSX2+ | OK | Operación verificada |
| Panasonic FS-A1WSX | MSX2+ | OK | Operación verificada |
| Sharp HotBit HB8000 | MSX1 | OK | Operación verificada |
| SMX-HB | MSX2+ (clon FPGA) | OK | Operación verificada |
| Sony HB-F1XD | MSX2 | OK | Operación verificada |
| Sony HB-F1XDJ | MSX2 | OK | Operación verificada |
| Sony HB-F1XV | MSX2+ | OK | Operación verificada |
| TRHMSX | MSX2+ (clon FPGA) | OK | Operación verificada |
| uMSX | MSX2+ (clon FPGA) | OK | Operación verificada |
| Yamaha YIS604 | MSX1 | OK | Operación verificada |

El cartucho PicoVerse 2040 con firmware Sunrise Nextor +240K (opción `-s2`) se ha probado en los siguientes modelos MSX:

| Modelo | Tipo | Estado | Comentarios |
| --- | --- | --- | --- |
| Gradiente Expert | MSX1 | Not OK | Mapeador de memoria no funciona, por tanto Nextor no funcional |
| Sharp HotBit HB8000 (MSX1) | MSX1| No OK | Mapeador de memoria no funciona, por tanto Nextor no funcional |
| Sony HB-F1XD | MSX2 | OK | Operación verificada |
| TRHMSX (MSX2+, clon FPGA) | MSX2+ | OK | Operación verificada |
| uMSX (MSX2+, clon FPGA) | MSX2+ | OK | Operación verificada |

La ROM Nextor embebida experimental (opción `-n`) ha sido probada en los siguientes modelos MSX:

| Modelo | Tipo | Estado | Comentarios |
| --- | --- | --- | --- |
| Omega MSX | MSX2+ | No OK | USB no detectado |
| Sony HB-F1XD | MSX2 | No OK | Error al leer un sector del disco |
| TRHMSX | MSX2+ (clon FPGA) | OK | Operación verificada |
| uMSX | MSX2+ (clon FPGA) | OK | Operación verificada |

Autor: Cristiano Almeida Goncalves
Última actualización: 25/12/2025
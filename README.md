# LABORATORIO DE CONTROL DE TEMPERATURA (TCLAB)

El Laboratorio de Control de Temperatura (TCLab) es una pequeña placa compatible con Arduino creada por John D. Hedengren y la comunidad APMonitor. Esta placa está diseñada para fines didácticos y después de probarla os puedo asegurar que es una virguería para los amantes del control. La placa en sí es muy sencilla, consta de dos sensores de temperatura y dos transistores usados como calentadores, lo realmente interesante es el software que lo acompaña ya que tenemos a nuestra disposición diferentes desarrollos en Matlab, Simulink y Python que van desde un simple PID hasta un controlador MPC no lineal, en definitiva, una maravilla con un potencial didáctico enorme.

En esta primera entrada vamos a obviar el software que acompaña a la placa usando mis propios desarrollos tanto en el firmware de Arduino como en el software de recolección de datos.

<h2>Características</h2>
La placa está compuesta por dos sensores de temperatura TMP36 con un rango de -40 a 150ºC y dos transistores TIP31C usados como calentadores. La transferencia entre sensor y calentador se da mediante conducción, convección y radiación, y como más adelante comprobaremos en la identificación, al estar tan juntas se afectan mucho entre sí.

<p align="center">
  <img src="https://garikoitz.info/blog/wp-content/uploads/2023/01/tclab_schematic_garikoitz-1-1024x723.png" width="350" alt="esquema">
</p>

Sensor y calentador están unidos por una pintura térmica de color gris que se vuelve rosa a medida que aumenta la temperatura. En la imagen que se puede ver a continuación T2 estaba aproximadamente a 50ºC.

<p align="center">
  <img src="https://garikoitz.info/blog/wp-content/uploads/2023/01/PXL_20230103_161130020-1024x768.jpg" width="350" alt="esquema">
</p>

Tenéis más información en la entrada del blog: https://garikoitz.info/blog/2023/01/sintonizar-pid-con-arduino-laboratorio-de-control-de-temperatura-tclab/

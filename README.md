# N3r0 was here
# Plataforma-robotica-baixo-custo
## Desenvolvimento de uma Plataforma Robótica de baixo custo para aplicação de técnicas avançadas em salas de aula de Graduação

Este robô foi desenvolvido para ser aplicado em salas de aula de graduação e para projetos educacionais. O objetivo principal é tornar acessível o acesso a robótica para estudantes universitários, levando em consideração a crescente aplicação dos conceitos desse campo no mercado de trabalho. Para que isso seja possível, a construção da plataforma robótica seleciona componentes de custo-benefício, tornando-a mais acessível que outras disponíveis no mercado. O robô foi submetido a testes reais, aproveitando ferramentas do ROS com a linguagem de programação Python.

**Códigos:**

BOB_wifi_ros.ino: Código que conecta a placa com todos os componentes do robô e gera os tópicos para o ROS.

desvio_obsaculos.py: Código que faz com que o robô ande para frente e desvia quando detecta a presença de algum  obstáculo.

servo_pub.py: Código que faz com que o servo motor se mova em posiçôes diferentes em 180° para auxiliar no desvio de obstáculos.

**Peças utilizadas na montagem do robô:**

| Peças       | Link        | 
| :----:      |    :----:   |  
| Micro Servo Motor         | [Mercado Livre](https://produto.mercadolivre.com.br/MLB-3139089695-micro-servo-motor-arduino-sg90-_JM?matt_tool=18956390&utm_source=google_shopping&utm_medium=organic)       |
| Sensor IR Sharp Gp2y0a21  | [Mercado Livre](https://produto.mercadolivre.com.br/MLB-2725727762-modulo-sensor-infravermelho-distncia-sharp-2y0a21-_JM?matt_tool=63064967&matt_word=&matt_source=google&matt_campaign_id=14303413826&matt_ad_group_id=133431076203&matt_match_type=&matt_network=g&matt_device=c&matt_creative=584156655540&matt_keyword=&matt_ad_position=&matt_ad_type=pla&matt_merchant_id=605492465&matt_product_id=MLB2725727762&matt_product_partition_id=310365260760&matt_target_id=aud-378637574381:pla-310365260760&gclid=CjwKCAiA3KefBhByEiwAi2LDHBhuD4J47TRn5wFEKvKcYRpqmiZtxhCRZEjo1YG3NIdEcd02zvl_EhoC_PMQAvD_BwE)       |
| Power Bank                | [Mercado Livre](https://produto.mercadolivre.com.br/MLB-3097623157-carregador-movel-portatil-power-bank-10000mah-promoco-_JM?matt_tool=88313147&matt_word=&matt_source=google&matt_campaign_id=14303413859&matt_ad_group_id=125984304197&matt_match_type=&matt_network=g&matt_device=c&matt_creative=539354957424&matt_keyword=&matt_ad_position=&matt_ad_type=pla&matt_merchant_id=678853133&matt_product_id=MLB3097623157&matt_product_partition_id=1799289347936&matt_target_id=aud-378637574381:pla-1799289347936&gclid=CjwKCAiA3KefBhByEiwAi2LDHDN6SpKcyvIrvGK6yaXjNz9N1VUz9Ib3p3hB20MXCpHJs1RcFNS5EBoC8vMQAvD_BwE)       |
| Cabos Jumper              | [Mercado Livre](https://produto.mercadolivre.com.br/MLB-2936455665-cabo-wire-jumper-20cm-40-fios-fmea-macho-protoboard-arduino-_JM?matt_tool=63064967&matt_word=&matt_source=google&matt_campaign_id=14303413826&matt_ad_group_id=133431076203&matt_match_type=&matt_network=g&matt_device=c&matt_creative=584156655540&matt_keyword=&matt_ad_position=&matt_ad_type=pla&matt_merchant_id=314188124&matt_product_id=MLB2936455665&matt_product_partition_id=310365260760&matt_target_id=pla-310365260760&gclid=Cj0KCQiAxbefBhDfARIsAL4XLRr9Rjej-Su__5GFSKbmZpdeVRAfr-R-NG_vvmce2fubYwsm89WrPSEaAgP7EALw_wcB)       |
| Ponte H                   | [Mercado Livre](https://produto.mercadolivre.com.br/MLB-1347075835-driver-motor-ponte-h-dupla-l298n-_JM?matt_tool=63064967&matt_word=&matt_source=google&matt_campaign_id=14303413826&matt_ad_group_id=133431076203&matt_match_type=&matt_network=g&matt_device=c&matt_creative=584156655540&matt_keyword=&matt_ad_position=&matt_ad_type=pla&matt_merchant_id=646268484&matt_product_id=MLB1347075835&matt_product_partition_id=310365260760&matt_target_id=aud-378637574381:pla-310365260760&gclid=CjwKCAiA_6yfBhBNEiwAkmXy59VJPwVsSh-KYDl0VE_o9OfLCCYq6A4oF58dyhtshKfCfPAvB4MtLBoCbWgQAvD_BwE)       |
| Filamento                 | [GTMAX3D](https://www.gtmax3d.com.br/filamentos/abs-premium)       |
| Placa WeMos D1            | [Mercado Livre](https://produto.mercadolivre.com.br/MLB-1654901596-placa-wemos-d1-r2-wifi-esp8266-ide-compativel-do-arduino-_JM#position=4&search_layout=grid&type=item&tracking_id=2409f693-e2d9-49e7-b8de-bc74dc06932d)       |
| Shield                    | [Shield](https://github.com/VitorAssis9/Plataforma-robotica-baixo-custo/tree/main/Shield)   |
| Kit 4 Motores DC          | [Mercado Livre](https://produto.mercadolivre.com.br/MLB-2030898738-4-x-motor-dc-3-6v-cx-reduco-e-eixo-duplo-roda-68mm-cnf-_JM#position=35&search_layout=grid&type=item&tracking_id=693da132-0130-410f-833b-c241a175b180)       |


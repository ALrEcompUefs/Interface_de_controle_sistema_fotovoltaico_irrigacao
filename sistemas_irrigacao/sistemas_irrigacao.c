#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <stdint.h>
#include "inc/font.h"
#include "string.h"
// Declaração de variavéis dos pinos    //

// definições para uso do display oled integrado
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// paramêtros padrão para o joystick
#define PADRAO_VRX 1900
#define PADRAO_VRY 2000

//valor máximo do contador - WRAP
#define WRAP_PERIOD  4096
//divisor do clock para o PWM 
#define PWM_DIVISER  25.4313151f 

// Pinos dos perifericos
const uint8_t LED_R=13, LED_B=12, LED_G=11;
const uint8_t BOTAO_A=5,BOTAO_B=6,BOTAO_JYK=22,Sensor_nivel=17;
const uint8_t VRX = 26, VRY=27;

// Variavel para registro de tempo e controle de bounce da interrupção
static volatile uint32_t tempo_anterior = 0;
// Variável de controle do nivel do tanque
static volatile bool tanque_cheio = false,bomba_ativa=false,ajuste_automatico=true,timer_programado=false;
// variavéis de nivel de pwm das saida
volatile uint16_t pwm_bomba=0, pwm_motor=0;

// variaveis de registro de consumo
uint nivel_bateria=0,nivel_agua=0,angulo=0, consumo_agua=0,tempo_ativacao=0,sensor_ldr=0;
// variável de registro de leitura do adc
static volatile uint16_t valor_x,valor_y;

// variável de registro do menu
static volatile uint menu=0;

// Inicializa a estrutura do display
ssd1306_t ssd; 

// declaração do cabeçalho de funções
void inicializar_leds();
void inicializar_botoes();
void inicializar_joystick();
void set_pwm_rgb(char cor,uint16_t nivel);
void inicializar_display_oled();
void atualizar_display();
static void gpio_irq_handler(uint gpio, uint32_t events);
void pwm_setup(uint8_t PINO);
void set_pwm_dc(uint16_t duty_cycle, uint8_t PINO);
void leitura_analogica();
void imprimir_menu();
uint16_t leitura_adc();
void funcao_menu();
void navegar_menu();
uint ajustar_posicao_modulos();
int64_t turn_off_callback(alarm_id_t id, void *user_data);
void programar_bomba(uint16_t tempo);

int main()
{
    // Inicialização dos perifericos
    stdio_init_all();
    inicializar_leds();
    inicializar_botoes();
    inicializar_joystick();
    inicializar_display_oled();
    adc_init();


    // configura interrupções
    gpio_set_irq_enabled_with_callback(BOTAO_A,GPIO_IRQ_EDGE_FALL,true,&gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B,GPIO_IRQ_EDGE_FALL,true,&gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_JYK,GPIO_IRQ_EDGE_FALL,true,&gpio_irq_handler);

    while (true) {
        // leitura da gpio 17
        tanque_cheio = gpio_get(Sensor_nivel);
        navegar_menu();
        imprimir_menu();
        if(ajuste_automatico){
            angulo=ajustar_posicao_modulos();
        }
        sleep_ms(200);
    }
}



/*
|   Função inicializar_leds
|   Configura os pinos da LED RGB como saída
*/
void inicializar_leds(){
    // Configura as leds como pwm
    pwm_setup(LED_R);
    pwm_setup(LED_B);
    pwm_setup(LED_G);
}

/*
|   Função de inicialização dos botões
|   Configura os botões A,B e do joystick como entrada em modo Pull-up
*/
void inicializar_botoes(){
    //botão A
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A,GPIO_IN);
    gpio_pull_up(BOTAO_A);
    //botão B
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B,GPIO_IN);
    gpio_pull_up(BOTAO_B);
    //botão C
    gpio_init(BOTAO_JYK);
    gpio_set_dir(BOTAO_JYK,GPIO_IN);
    gpio_pull_up(BOTAO_JYK);
    // configura o pino 17 do GPIO como uma entrada
    gpio_init(Sensor_nivel);
    gpio_set_dir(Sensor_nivel,GPIO_IN);
    gpio_pull_up(Sensor_nivel);
}


void inicializar_joystick(){
    // Inicializa pinos do joystick como pinos do ADC
    adc_gpio_init(VRX);
    adc_gpio_init(VRY);
    //botão do joystick
    gpio_init(BOTAO_JYK);
    gpio_set_dir(BOTAO_JYK,GPIO_IN);
    gpio_pull_up(BOTAO_JYK);
}

/*
|   Função inicialzar display oled
|   Configura e inicializa o display oled ssd1306 para sua utilização
|   A comunicação é feita utilizando i2c
*/
void inicializar_display_oled(){
    // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  //ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

}


/*
|   Função gpio_irq_handler
|   Função de callback para tratamento de interrupção da GPIO
|   Implementa um debouncer via software e controla o estado das leds azul e verde
|   além de ativar o modo bootsel
*/
static void gpio_irq_handler(uint gpio, uint32_t events){
    // obtém tempo atual da execução do programa
    uint32_t tempo_atual = to_us_since_boot(get_absolute_time());

    // com o botão pressionado por pelo menos 200ms
    if(tempo_atual-tempo_anterior > 200000){
        tempo_anterior= tempo_atual;
        // executa tratamento da interrupção
        if(gpio == BOTAO_A){
            // veririfica qual função do menu executar
            funcao_menu();
        }
        else if(gpio == BOTAO_B){
            reset_usb_boot(0,0);
        }
        else if(gpio == BOTAO_JYK){
            leitura_analogica();
            
        }
    }
}


//função para configurar o módulo PWM
void pwm_setup(uint8_t PINO)
{
    // Com o clock base de 125Mhz WRAP de 10000 e divisor de 125
    // O clock calculado para o pwm é de 50Hz
    // outras configurações de parâmetro poderiam ser utilizadas para obter esta mesma frequência
    // Mas os requisitos da tarefa limitaram a escolha do valor do wrap

    gpio_set_function(PINO, GPIO_FUNC_PWM); //habilitar o pino GPIO como PWM

    uint slice = pwm_gpio_to_slice_num(PINO); //obter o canal PWM da GPIO

    pwm_set_wrap(slice, WRAP_PERIOD); //definir o valor de wrap
    pwm_set_clkdiv(slice, PWM_DIVISER); //define o divisor de clock do PWM
    pwm_set_enabled(slice, true); //habilita o pwm no slice correspondente
}

/*
|   Função set_pwm_dc
|   Configura o nível do duty cycle para o pwm
*/
void set_pwm_dc(uint16_t duty_cycle, uint8_t PINO){
    uint slice = pwm_gpio_to_slice_num(PINO);
    pwm_set_gpio_level(PINO, duty_cycle); //definir o cico de trabalho (duty cycle) do pwm
}

void set_pwm_rgb(char cor,uint16_t nivel){
        switch (cor)
        {
        case 'R':
            set_pwm_dc(nivel,LED_R);
            set_pwm_dc(0,LED_G);
            set_pwm_dc(0,LED_B);
            break;
        case 'G':
            set_pwm_dc(0,LED_R);
            set_pwm_dc(nivel,LED_G);
            set_pwm_dc(0,LED_B);
        break;
        case 'B':
            set_pwm_dc(0,LED_R);
            set_pwm_dc(0,LED_G);
            set_pwm_dc(nivel,LED_B);
        break;
        default:
            break;
        }
}

/*
|   Função imprimir_menu
|   Exibe as opções de menu no display oled
*/
void imprimir_menu(){
    uint16_t leitura=0;
    float aux;
    char str[20];
    ssd1306_fill(&ssd, false); // Limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
    switch (menu){
    // nivel da bateria e do tanque
    case 0:
        snprintf(str, sizeof(str), "%u",nivel_bateria);
        ssd1306_draw_string(&ssd, "Bateria", 10, 8); // Desenha uma string
        ssd1306_draw_string(&ssd, str, 10, 16); // Desenha uma string
        ssd1306_draw_string(&ssd, "Tanque", 10, 24); // Desenha uma string
        if(tanque_cheio)
            ssd1306_draw_string(&ssd, "Nivel maximo", 20, 32); // Desenha uma string
        else
            ssd1306_draw_string(&ssd, "nivel baixo", 20, 32); // Desenha uma string
        ssd1306_send_data(&ssd); // Atualiza o display
        break;
    // Leitura do sensor de fluxo
    case 1:
        if(bomba_ativa){
            leitura = leitura_adc();
            consumo_agua +=  (leitura/4096.0)*100;
            snprintf(str, sizeof(str), "%u",leitura);
            ssd1306_draw_string(&ssd, "sensor fluxo", 10, 8); // Desenha uma string
            ssd1306_draw_string(&ssd, str, 10, 16); // Desenha uma string
            ssd1306_draw_string(&ssd, "Consumo total", 10, 16); // Desenha uma string
            snprintf(str, sizeof(str), "%u",consumo_agua);
            ssd1306_draw_string(&ssd, str, 10, 32); // Desenha uma string
            ssd1306_send_data(&ssd); // Atualiza o display

            // ativa saida no led rgb da bomba
            set_pwm_rgb('B',pwm_bomba);
        }else{
            ssd1306_draw_string(&ssd, "Consumo total", 10, 8); // Desenha uma string
            snprintf(str, sizeof(str), "%u",consumo_agua);
            ssd1306_draw_string(&ssd, str, 10, 16); // Desenha uma string
            ssd1306_send_data(&ssd); // Atualiza o display
            set_pwm_rgb('B',0);
        }
        break;
    // ativar bomba
    case 2:
        if(bomba_ativa){
            ssd1306_draw_string(&ssd, "Desativar bomba", 10, 8); // Desenha uma string
            ssd1306_draw_string(&ssd, "BOTAO A", 10, 16); // Desenha uma string
            set_pwm_rgb('B',pwm_bomba);
        }
        else{
            ssd1306_draw_string(&ssd, "Ativar bomba", 10, 8); // Desenha uma string
            ssd1306_draw_string(&ssd, "BOTAO A", 10, 16); // Desenha uma string
            set_pwm_rgb('B',0);
        }
        ssd1306_send_data(&ssd); // Atualiza o display
        break;
    // programar bomba
    case 3:
        leitura = leitura_adc();
        tempo_ativacao = (leitura/4096.0)*10.0;
        snprintf(str, sizeof(str), "%u",tempo_ativacao);
        ssd1306_draw_string(&ssd, "Programar bomba", 5, 8); // Desenha uma string
        ssd1306_draw_string(&ssd, "Para", 10, 16); // Desenha uma string
        ssd1306_draw_string(&ssd, str, 10, 24); // Desenha uma string
        ssd1306_send_data(&ssd); // Atualiza o display
        break;
    // ativar/desativar ajuste automatico dos módulos
    case 4:
        if(ajuste_automatico){
            ssd1306_draw_string(&ssd, "Desativar", 10, 8); // Desenha uma string
            ssd1306_draw_string(&ssd, "Ajuste", 10, 16); // Desenha uma string
            ssd1306_draw_string(&ssd, "BOTAO A", 10, 24); // Desenha uma string
        }
        else{
            ssd1306_draw_string(&ssd, "Ativar", 10, 8); // Desenha uma string
            ssd1306_draw_string(&ssd, "Ajuste", 8, 16); // Desenha uma string
            ssd1306_draw_string(&ssd, "BOTAO A", 10, 24); // Desenha uma string
        }
        ssd1306_draw_string(&ssd, "Angulo atual", 10, 32); // Desenha uma string
        snprintf(str, sizeof(str), "%u",angulo);
        ssd1306_draw_string(&ssd, str, 10, 40); 
        ssd1306_send_data(&ssd); // Atualiza o display

        // exibe saida pwm no motor
        set_pwm_rgb('R',pwm_motor);
        break;
    //  ajustar módulos
    case 5:
        if(ajuste_automatico){
            ssd1306_draw_string(&ssd, "Ajuste manual", 10, 8); // Desenha uma string
            ssd1306_draw_string(&ssd, "BLOQUEADO", 10, 16); // Desenha uma string
        }
        else{
            leitura = leitura_adc();
            leitura = (leitura/4096.0)*180.0;
            snprintf(str, sizeof(str), "%u",leitura);
            ssd1306_draw_string(&ssd, "Novo angulo", 10, 8); // Desenha uma string
            ssd1306_draw_string(&ssd, str, 10, 16); // Desenha uma string
        }
        ssd1306_send_data(&ssd); // Atualiza o display
        set_pwm_rgb('R',pwm_motor);
        break;
    default:
        break;
    }
}

/*
|   Função leitura ánalogica
|   Atualiza o valor análogico com base no valor medido no eixo y do joystick
*/
void leitura_analogica(){
    uint16_t leitura_x=0;
    switch (menu)
    {
    case 0:
        // atualiza nivel da bateria
        leitura_x =leitura_adc();
        // converte para um valor de 0 a 100
        leitura_x = ( leitura_x/4096.0)*100;
        nivel_bateria = leitura_x;
        break;
    case 2:
        // intensidade da bomba
        leitura_x =leitura_adc();
        // converte para um valor de 0 a 100
        pwm_bomba = leitura_x;
        break;
    case 3:
        // intensidade da bomba
        leitura_x =leitura_adc();
        // converte para um valor de 0 a 100
        pwm_bomba = leitura_x;
        break;
    default:
    case 4:
        // atualiza posicao do sol
        leitura_x =leitura_adc();
        pwm_motor = leitura_x;
        // converte para um valor de 0 a 100
        leitura_x = ( leitura_x/4096.0)*240;
        sensor_ldr = leitura_x;
    break;
    case 5:
        // atualiza posicao do sol
        leitura_x =leitura_adc();
        pwm_motor = leitura_x;
        // converte para um valor de 0 a 100
        leitura_x = ( leitura_x/4096.0)*180;
        angulo = leitura_x;
        break;
    }
}

/*
|   Função leitura_adc
|   Realiza a leitura do valor analogico do eixo x de joystick e a conversão para o valor digital
*/
uint16_t leitura_adc(){
    // leitura do canal 0- eixo x do joystick
    adc_select_input(0);
    sleep_us(2);
    return adc_read();
}

/*
|   Função funcao_menu
|   Realiza a função de cada item do menu
*/
void funcao_menu(){
    switch(menu){
        case 2:
            if(!timer_programado)
                bomba_ativa= !bomba_ativa;
            break;
        case 3:
            programar_bomba(tempo_ativacao);
            break;
        case 4:
            ajuste_automatico = !ajuste_automatico;
            break;
        
    }
}

/*
|   Função navegar_menu
|   Atuializa o menu visto pelo usuario
*/
void navegar_menu(){
    uint16_t leitura;
    // leitura do canal 1- eixo y do joystick
    adc_select_input(1);
    sleep_us(2);
    leitura = adc_read();

    // navega o menu para os lados
    if(leitura > 3000)
        menu++;
    else if(leitura <1800)
        menu--;
    
    // veririca se passou dos limites
    if(menu<0)
        menu=5;
    if(menu>5)
        menu=0;
}

/*
|   Função ajustar_psocao_modulos
|   Ajusta a posicao dos modulos com base no dados do sensor ldr
*/
uint ajustar_posicao_modulos(){
    uint novo_angulo;

    if(sensor_ldr > 200){
        novo_angulo = 180;
    }
    else if(sensor_ldr >= 145 &&  sensor_ldr <= 180){
        novo_angulo= 120;
    }
    else if(sensor_ldr >= 90 &&  sensor_ldr <= 120){
        novo_angulo = 90;
    }
    else if(sensor_ldr >= 60 &&  sensor_ldr < 90){
        novo_angulo = 60;
    }
    else if(sensor_ldr >= 45 &&  sensor_ldr < 60){
        novo_angulo = 45;
    }
    else{
        novo_angulo = 30;
    }
    return novo_angulo;
}

/* 
|   Função de callback para desligar o LED após o tempo programado.
|   O retorno da função define se o alarme vai rearmado ou não
|   Se o retorno for 0 o alarme não é rearmado
|   Com um retorno em valor maior que zero, o alarme é rearmado com o tempo
|   passado pelo retorno
|*/
int64_t turn_off_callback(alarm_id_t id, void *user_data) {
    // desativa a bomba após o tempo estourar
    bomba_ativa = false;
    timer_programado =false;
    set_pwm_rgb('B',0);
    return 0;
}


/*
|   Função programar_bomba
|   Configura um alarme que ativa a bomba durante este intervalo
*/
void programar_bomba(uint16_t tempo){
    timer_programado = true;
    bomba_ativa =true;
    set_pwm_rgb('B',pwm_bomba);
    add_alarm_in_ms(tempo*1000, turn_off_callback, NULL, false);
}
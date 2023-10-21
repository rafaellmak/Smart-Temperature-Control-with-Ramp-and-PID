# Nome do arquivo de objeto
obj-m += my_max6675_ramp.o

# Configuração
my_max6675_ramp-objs := my_max6675_ramp1.o my_max6675_ramp2.o

# Regra de construção
all:
    make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

# Regra de instalação do módulo
install:
    make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

# Regra de limpeza
clean:
    make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean



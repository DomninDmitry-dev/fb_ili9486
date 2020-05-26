
ifneq ($(KERNELRELEASE),)

obj-m := ${MODNAME}.o

else

# Добавляем -DDEBUG, если в конфигурации ядра 
# отключена опция CONFIG_DYNAMIC_DEBUG
#CFLAGS_$(MODNAME).o := -DDEBUG

.PHONY: default clean

default: ${MODNAME}.c
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) clean

endif

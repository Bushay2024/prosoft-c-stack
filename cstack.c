#include "cstack.h"
#include <stdlib.h>
#include <string.h>

// Все стэки мы будем держать в массиве фиксированной длины
// В качестве хэндлера стэка будем использовать его индекс в этом массиве
// Если какой-то стек уничтожается, то освободившийся хэндрер можно переиспользовать
// под новый стек

#define MAX_STACKS 100

// Стркутура, представляющая элемента стека
// Содержит указатель на нижележащий элемент, указатель на данные и размер этих данных
struct stack_item
{
    struct stack_item* next;
    unsigned int size;
    char data[0];
};

// Стэк - список элементов; в структуре мы храним указатель на головной элемент,
// общее число элементов и хэндл
struct stack
{
    struct stack_item* head;
    unsigned int size;
    hstack_t handler;
};

// массив всех стэков - фиксированной длины
struct stack* all_stacks[MAX_STACKS] = { NULL };

hstack_t stack_new(void)
{
    // В качестве хэндлера стэка будем использовать его индекс в массиве all_stacks
    // При создании нового стэка - ищем первый незанятый элемент в этом массиве
    // и создаем стэк там
    for (unsigned int i=0; i < MAX_STACKS; i++) {
        if (all_stacks[i] == NULL) {
            all_stacks[i] = (struct stack*)malloc(sizeof(struct stack));
            if (all_stacks[i] == NULL) {
                return -1;
            }
            all_stacks[i]->size = 0;
            all_stacks[i]->head = NULL;
            return i;
        }
    }
    return -1;
}

void stack_free(const hstack_t hstack)
{
    if (stack_valid_handler(hstack) == 0) {
        // Проходим по всем элементам стека, начиная с головы
        struct stack_item* item = all_stacks[hstack]->head;
        while (item != NULL) {
            // Освобождаем текущий элемент и переходим к следующему
            all_stacks[hstack]->head = all_stacks[hstack]->head->next;
            free(item);
            item = all_stacks[hstack]->head;
        }
        free(all_stacks[hstack]);
        // Помечаем место в массиве all_stacks как свободное
        all_stacks[hstack] = NULL;
    }
}

int stack_valid_handler(const hstack_t hstack)
{
    // Валидный хэндл - если он меньше размера массива
    // и если по его адресу в массиве что-то есть
    if (hstack >= MAX_STACKS || hstack < 0 ) {
        return 1;
    }
    if (all_stacks[hstack] != NULL) {
        return 0;
    }
    return 1;
}

unsigned int stack_size(const hstack_t hstack)
{
    // Если хэндл валидный - то достаточно вернуть поле size соответсвующего стэка, иначе - 0
    
    if (stack_valid_handler(hstack) == 0) {
        return all_stacks[hstack]->size;
    }
    return 0;
}

void stack_push(const hstack_t hstack, const void* data_in, const unsigned int size)
{
    if (stack_valid_handler(hstack)) {
        return;
    }
    if (data_in == NULL || size == 0) {
        return;
    }

    // Если с входными данными все ок, то создаем новый элемент стека.
    // В качестве нижележащего указываем текущую голову стека, а потом сдвигаем эту голову
    // на новый элемент
    struct stack_item* new_item = (struct stack_item*)malloc(sizeof(struct stack_item) + size);
    if (!new_item) {
        return;
    }

    new_item->next = all_stacks[hstack]->head;
    new_item->size = size;
    memcpy(new_item->data, data_in, size);
    all_stacks[hstack]->head = new_item;
    // Не забeдем увеличить число элементов в стеке
    all_stacks[hstack]->size++;
}

unsigned int stack_pop(const hstack_t hstack, void* data_out, const unsigned int size)
{
    // Хэндл валидный?
    if (stack_valid_handler(hstack)) {
        return 0;
    }
    // Размер стэка ненулеваой?
    if (all_stacks[hstack]->size == 0) {
        return 0;
    }
    // Нам передали корректные данные для выгрузки элемента?
    if (data_out == NULL || size == 0 || size < all_stacks[hstack]->head->size) {
        return 0;
    }

    // Копируем  верхний элемент стэка в data_out
    struct stack_item* item = all_stacks[hstack]->head;
    memcpy(data_out, item->data, size);
    all_stacks[hstack]->head = item->next;
    all_stacks[hstack]->size--;
    free(item);

    // возвращаем его размер
    return size;
}

#include "message_center.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_log.h"

/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
    .topic_name      = "Message_Manager",
    .first_subs      = NULL,
    .next_topic_node = NULL};

static void CheckName(char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN) {
        // LOGERROR("EVENT NAME TOO LONG:%s", name);
        (void)0;
        while (1)
            ; // 进入这里说明话题名超出长度限制
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2) {
        // LOGERROR("EVENT LEN NOT SAME:%d,%d", len1, len2);
        (void)0;
        while (1)
            ; // 进入这里说明相同话题的消息长度却不同
    }
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    CheckName(name);
    Publisher_t *node = &message_center;
    while (node->next_topic_node) // message_center会直接跳过,不需要特殊处理,这被称作dumb_head(编程技巧)
    {
        node = node->next_topic_node;            // 切换到下一个发布者(话题)结点
        if (strcmp(node->topic_name, name) == 0) // 如果已经注册了相同的话题,直接返回结点指针
        {
            CheckLen(data_len, node->data_len);
            node->pub_registered_flag = 1;
            return node;
        }
    } // 遍历完发现尚未创建name对应的话题
    // 在链表尾部创建新的话题并初始化
    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->pub_registered_flag = 1;
    return node->next_topic_node;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    Publisher_t *pub = PubRegister(name, data_len); // 查找或创建该话题的发布者
    // 创建新的订阅者结点,申请内存,注意要memset因为新空间不一定是空的,可能有之前留存的垃圾值
    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    memset(ret, 0, sizeof(Subscriber_t));
    ret->data_len = data_len;
    // 用 FreeRTOS 队列代替手写 ring buffer; xQueueOverwrite 提供与原代码相同的"满则覆盖"语义
    ret->queue = xQueueCreate(QUEUE_SIZE, data_len);
    configASSERT(ret->queue != NULL);
    // 如果是第一个订阅者,特殊处理一下,将first_subs指针指向新建的订阅者(详见文档)
    if (pub->first_subs == NULL) {
        pub->first_subs = ret;
        return ret;
    }
    // 若该话题已经有订阅者, 遍历订阅者链表,直到到达尾部
    Subscriber_t *sub = pub->first_subs; // 作为iterator
    while (sub->next_subs_queue)         // 遍历订阅了该话题的订阅者链表
    {
        sub = sub->next_subs_queue; // 移动到下一个订阅者,遇到空指针停下,说明到了链表尾部
    }
    sub->next_subs_queue = ret; // 把刚刚创建的订阅者接上
    return ret;
}

/* 如果队列为空,会返回0;成功获取数据,返回1 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    return xQueueReceive(sub->queue, data_ptr, 0) == pdPASS ? 1 : 0;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    Subscriber_t *iter = pub->first_subs;
    while (iter) {
        xQueueOverwrite(iter->queue, data_ptr);
        iter = iter->next_subs_queue;
    }
    return 1;
}

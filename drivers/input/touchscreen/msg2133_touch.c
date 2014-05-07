#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
//#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <linux/input/msg2133_touch.h>
#include <linux/slab.h>
#include <linux/device.h>

#define MSG2133_ABS_X_MIN	0
#define MSG2133_ABS_X_MAX 	2047
#define MSG2133_ABS_Y_MIN	0
#define MSG2133_ABS_Y_MAX	2047

#define MAX_SUPPORT_POINTS	6	/* Max supported keys */

#define MAX_SUPPORT_CONTACTS	2	/* Max supported fingers */

#define MSG2133_QUERY_SIZE 8
#define REPORT_BIT_PRESSED	(1 << 0)	


struct msg2133_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct msg2133_platform_data *pdata;
	unsigned int irq_gpio;
	unsigned int reset_gpio;	
	unsigned int irq_active_high;
	int irq;
 	struct work_struct msg2133_work;
	struct mutex mutex;
	struct i2c_adapter *adapter;
	
};

struct touch_pos {
	unsigned short pos1_x:11;
	unsigned short pos1_y:11;
	unsigned short pos2_x:11;
	unsigned short pos2_y:11;
	
};


static inline int msg2133_ts_irq_active(struct msg2133_data *data)
{
	return gpio_get_value(data->irq_gpio) == data->irq_active_high;
}

u8 calculate_8bitchecksum(u8 *msg, u32 length)
{
	u32 check_sum = 0;
	u32 i;

	for(i = 0; i < length; i++)
	{
		check_sum += msg[i];	
	}	
	return (u8) (( -check_sum) & 0xFF);
}

static void msg2133_ts_reset(struct msg2133_data *data)
{
        gpio_set_value(data->reset_gpio, 0);
        gpio_set_value(data->reset_gpio, 1);

}

static void msg2133_ts_read(struct work_struct *work)
{
	struct touch_pos pos;
	char buf[8];
	unsigned int ret, pressure;
	struct msg2133_data *data = container_of(work, struct msg2133_data, msg2133_work);
	mutex_lock(&data->mutex);
	struct i2c_msg msgs[] = {
		{
			.addr	= 0xc1,
			.flags	= I2C_M_RD,
			.len	= sizeof(buf),
			.buf	= buf,
		}
	};
	udelay(100);
		ret = i2c_transfer(data->client->adapter, msgs, 1);
		if(ret < 0)
			printk(KERN_ERR "Unable to clear IRQ\n");

	if(buf[7] == calculate_8bitchecksum(buf, 7)) {
		pos.pos1_x = (((buf[1] & 0xF0) << 4) | (buf[2]));
                pos.pos1_y = (((buf[1] & 0x0F) << 8) | (buf[3]));
		pressure = buf[0] & REPORT_BIT_PRESSED;

		input_report_abs(data->input, ABS_X, pos.pos1_x);
        	input_report_abs(data->input, ABS_Y, pos.pos1_y);
		input_report_key(data->input, ABS_PRESSURE, pressure);
		input_sync(data->input);
				
	}
#ifdef __hcit__
	if(buf[7] == calculate_8bitchecksum(buf, 7)) {
		pos.pos1_x = (((buf[1] & 0xF0) << 4) | (buf[2]));
                pos.pos1_y = (((buf[1] & 0x0F) << 8) | (buf[3]));
                pos.pos2_x = (((buf[4] & 0xF0) << 4) | (buf[5]));
                pos.pos2_y = (((buf[4] & 0x0F) << 8) | (buf[6]));
		pos.touch_contact = (buf[0] & 0x30);
		if(((buf[0] &0xc0) >> 6) == 0x02) {
			switch((buf[0] & 0x3f)) {
			case 0x01:
				pos.key_contact = 1;
				break;
			case 0x02:	
				pos.key_contact = 2;
				break;
			case 0x04:
				pos.key_contact = 3;
				break;
			case 0x08:
				pos.key_contact = 4;
				break;
			case 0x10:
				pos.key_contact = 5;
				break;
			case 0x20:
				pos.key_contact = 6;
				break;
			default:
				pos.key_contact = 0;
				break;
			}
		}
		if(((buf[0] & 0xc0) >> 6) == 0x01) {	/* Single finger touch */
			if(pos.touch_contact == 1) {
				input_report_abs(data->input, ABS_X, (pos.pos1_x + pos.pos2_x));
        			input_report_abs(data->input, ABS_Y, (pos.pos1_y + pos.pos2_y));
				input_report_key(data->input, BTN_TOUCH, pos.key_contact);
				input_sync(data->input);
				
			} else if(pos.touch_contact == 2) {			/* two finger touch */
				input_report_abs(data->input, ABS_X, pos.pos1_x);
        			input_report_abs(data->input, ABS_Y, pos.pos1_y);
				input_report_abs(data->input, ABS_X, pos.pos2_x);
        			input_report_abs(data->input, ABS_Y, pos.pos2_y);
				input_report_key(data->input, BTN_TOUCH, pos.key_contact);
				input_sync(data->input);
			} else {
				input_report_key(data->input, BTN_TOUCH, pos.key_contact);
				input_sync(data->input);
			}
		}
//		printk(KERN_ERR"8-bit Checksum is matched\n");
	}
	//msg2133_ts_reset(data);

#endif
	mutex_unlock(&data->mutex);
	msg2133_ts_reset(data);
	mdelay(100);
}

static int msg2133_ts_open(struct input_dev *dev)
{
	return 0;
}

static int msg2133_ts_close(struct input_dev *dev)
{
	return 0;
}

static irqreturn_t msg2133_ts_irq(int irq, void *dev_id)
{
	struct msg2133_data *data = dev_id;

	schedule_work(&data->msg2133_work);

	return IRQ_HANDLED;
}

static int msg2133_ts_probe(struct i2c_client *client, 
				const struct i2c_device_id *idp)
{
	struct msg2133_platform_data *pdata = client->dev.platform_data;
	struct msg2133_data *data;
	struct input_dev *input;
	int error;
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if(data == NULL) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	mutex_init(&data->mutex);
	input = input_allocate_device();		
	if(input == NULL) {                               
		dev_err(&client->dev," failed to allocate input device\n");
		input_free_device(input);
        	kfree(data);
		return -ENOMEM;
	}		
	
	error = gpio_request(pdata->reset_gpio, "msg2133_reset");
	if(error < 0) {
		dev_err(&client->dev, "msg2133_probe gpio_request (reset) failed \n");
		gpio_free(pdata->reset_gpio);
		return -EIO;
	}

	data->reset_gpio = pdata->reset_gpio;
	
	error = gpio_direction_output(data->reset_gpio, 1);
	if(error < 0) {
		dev_err(&client->dev, "msg2133_probe gpio_direction_output (reset) failed \n");
		gpio_free(pdata->reset_gpio);
		return -EIO;
	}

	error = gpio_request(pdata->irq_gpio, "msg2133_irq");
	if(error) {
		dev_err(&client->dev, "msg2133_probe gpio_request (irq) failed \n");
		gpio_free(pdata->irq_gpio);
		return -EIO;
	}
	data->irq_gpio = pdata->irq_gpio;
	error = gpio_direction_input(data->irq_gpio);
	if(error) {
		dev_err(&client->dev, "msg2133_probe gpio_direction_input (irq) failed \n");
		gpio_free(pdata->irq_gpio);
		return -EIO;
	}


	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#ifndef MSG2133_MULTI_TOUCH
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input->absbit[0] = BIT(ABS_X) | BIT(ABS_Y);

        input_set_abs_params(input, ABS_X, MSG2133_ABS_X_MIN, MSG2133_ABS_X_MAX, 0, 0);
        input_set_abs_params(input, ABS_Y, MSG2133_ABS_Y_MIN, MSG2133_ABS_Y_MAX, 0, 0);
        input_set_abs_params(input, ABS_PRESSURE, 0, 0XFF, 0, 0);
#else
        input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 0xFF, 0, 0);
        input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
        input_set_abs_params(input, ABS_MT_POSITION_X, MSG2133_ABS_X_MIN, MSG2133_ABS_X_MAX, 0, 0);
        input_set_abs_params(input, ABS_MT_POSITION_y, MSG2133_ABS_Y_MIN, MSG2133_ABS_Y_MAX, 0, 0);
	input_mt_init_slots(intput, MAX_SUPPORT_CONTACTS);
#endif
	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->phys = "msg2133/event0";
	input->dev.parent = &client->dev;
	input->open = msg2133_ts_open;
	input->close = msg2133_ts_close;

	data->client = client;
	data->input = input;
	data->pdata = pdata;
	data->adapter = client->adapter;
	data->irq = gpio_to_irq(data->irq_gpio);
	data->irq_active_high = pdata->irq_active_high;	

	INIT_WORK(&data->msg2133_work, msg2133_ts_read);
	i2c_set_clientdata(client, data);
	input_set_drvdata(input, data);
	
	error = input_register_device(input);
	if(error) {
		dev_err(&client->dev, "failed to register input device\n");
		input_free_device(input);
        	i2c_set_clientdata(client, NULL);
        	kfree(data);
		return -ENODEV;
	}

	error = request_irq(data->irq, msg2133_ts_irq, IRQF_DISABLED, client->name, data);
	if(error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		input_unregister_device(input);	
	}	

	device_init_wakeup(&client->dev, 1);
	
	return 0;
}

static int __devexit msg2133_ts_remove(struct i2c_client *client)
{
	struct msg2133_data *data = i2c_get_clientdata(client);
	if(!data)
		return -ENOMEM;
	free_irq(data->irq, data);
	gpio_free(data->reset_gpio);
	gpio_free(data->irq_gpio);
	input_unregister_device(data->input);
	i2c_set_clientdata(client, NULL);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int msg2133_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct msg2133_data *data = i2c_get_clientdata(client);

	 if (device_may_wakeup(&client->dev))
                enable_irq_wake(data->irq);

	return 0;
}

static int msg2133_ts_resume(struct i2c_client *client)
{
	struct msg2133_data *data = i2c_get_clientdata(client);
	
	if (device_may_wakeup(&client->dev))
                disable_irq_wake(data->irq);

	return 0;
}
#else
#define msg2133_ts_suspend NULL
#define msg2133_ts_resume NULL
#endif
static const struct i2c_device_id msg2133_ts_id[] = {
	{"msg2133", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, msg2133_ts_id);

static struct i2c_driver msg2133_ts_driver = {
	.driver = {
		.name 	= "msg2133",
	},
	.probe		= msg2133_ts_probe,
	.remove		= __devexit_p(msg2133_ts_remove),
	.suspend	= msg2133_ts_suspend,
	.resume		= msg2133_ts_resume,
	.id_table 	= msg2133_ts_id,
};

#ifndef __hcit__

static int __init msg2133_init(void)
{	
	printk(KERN_ERR "msg2133_init() called\n");
	int ret = i2c_add_driver(&msg2133_ts_driver);
	if(ret) {

		printk(KERN_WARNING "i2c_add_driver failed\n");
		return ret;
	}	
	
	printk(KERN_ERR "msg2133_init() ended\n");
	return 0;
}

static void __exit msg2133_exit(void)
{
	printk(KERN_ERR "msg2133_exit() called\n");
	i2c_del_driver(&msg2133_ts_driver);
	printk(KERN_ERR "msg2133_exit() ended\n");	
}

module_init(msg2133_init);
module_exit(msg2133_exit);

#endif
//module_i2c_driver(msg2133_ts_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("vasubabu <k.vasubabu@hcitechnocrats.com>");
MODULE_DESCRIPTION("MSG2133 I2C interface touch controller driver ");

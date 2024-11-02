(function ($) {

    function reset_form(selector) {

        selector.find('input[type="text"]').val('');
        selector.find('input[type="email"]').val('');
        selector.find('input[type="password"]').val('');
        selector.find('input[type="number"]').val('');
        selector.find('textarea').val('');
        selector.find('input[type="checkbox"]').removeAttr('checked');
        selector.find('option').removeAttr('selected');
        selector.find('.sbHolder > a:nth-child(2)').each(function () {
            var replace_html = $(this).parent().find('ul > li > a').html();
            $(this).html(replace_html);
        });
        if ($(selector).find('#g-recaptcha-response').length > 0) {
            grecaptcha.reset();
        }
    }

    $(function () {
        $('.ufbl-front-form').submit(function (e) {
            e.preventDefault();
            var selector = $(this);
            var form_data = selector.serializeArray();
            var captchaResponse = $(this).find('#g-recaptcha-response').val();
            $.ajax({
                url: frontend_js_obj.ajax_url,
                type: 'post',
                data: {
                    _wpnonce: frontend_js_obj.ajax_nonce,
                    form_data: form_data,
                    captchaResponse: captchaResponse,
                    action: 'ufbl_front_form_action'

                },
                beforeSend: function () {
                    selector.find('.ufbl-form-loader').show();
                    selector.find('.ufbl-error').html('');
                    $('.ufbl-form-message').slideUp(300);
                },
                success: function (res) {
                    selector.find('.ufbl-form-loader').hide();
                    console.log(res);
                    res = $.parseJSON(res);
                    if (res.error_flag == 1) {
                        for (error_key in res.error_keys) {
                            selector.find('div[data-error-key="' + error_key + '"]').html(res.error_keys[error_key]);
                        }
                        selector.find('.ufbl-form-message').html(res.response_message).removeClass('ufbl-success-message').addClass('ufbl-error-message').slideDown(300);
                        if ($(selector).find('#g-recaptcha-response').length > 0) {
                            grecaptcha.reset();
                        }
                    } else {
                        selector.find('.ufbl-form-message').html(res.response_message).removeClass('ufbl-error-message').addClass('ufbl-success-message').slideDown(300);
                        selector.find('.ufbl-error').html('');
                        reset_form(selector);

                    }
                }
            });

        });

        $('.ufbl-front-form input[type="text"],.ufbl-front-form input[type="email"],.ufbl-front-form input[type="email"],.ufbl-front-form textarea,.ufbl-front-form input[type="password"]').keyup(function () {
            $(this).next('.ufbl-error').html('');
        });
        $('.ufbl-front-form input[type="radio"],.ufbl-front-form input[type="checkbox"]').click(function () {
            $(this).closest('.ufbl-form-field').find('.ufbl-error').html('');
        });
        $('.ufbl-front-form select').change(function () {
            $(this).closest('.ufbl-form-field').find('.ufbl-error').html('');
        });

        $('.ufbl-math-captcha-ans').keyup(function () {
            $(this).closest('.ufbl-form-field-wrap').find('.ufbl-error').html('');
        });

        /**
         * Custom dropdown
         */
        $(".ufbl-template-1 .ufbl-form-dropdown,.ufbl-template-2 .ufbl-form-dropdown,.ufbl-template-3 .ufbl-form-dropdown,.ufbl-template-4 .ufbl-form-dropdown,.ufbl-template-5 .ufbl-form-dropdown").selectbox();

        $('.ufbl-form-reset').click(function (e) {
            //e.preventDefault();
            var selector = $(this).closest('form');
            reset_form(selector);
        });


    });//document.ready close
}(jQuery));

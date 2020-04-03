import django.forms

class GotoForm(django.forms.Form):
    dec = django.forms.FloatField(max_value=90, min_value=-90, widget=django.forms.NumberInput(attrs={'id':'goto_form_dec', 'required':True, 'placeholder':'in format DD.DDD'}))
    hr = django.forms.FloatField(max_value=360, min_value=-360, widget=django.forms.NumberInput(attrs={'id':'goto_form_hr', 'required':True, 'placeholder':'in format DD.DDD'}))
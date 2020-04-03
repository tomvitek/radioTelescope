from django.shortcuts import render
from django.http import HttpResponse, HttpResponseRedirect
from django.urls import reverse
from . import urls, models
from .ratelBridge import RatelBridge, Position
from .forms import GotoForm

ratel = RatelBridge.getInstance()
# Create your views here.
def home(request):
    goto_form = GotoForm
    context = {
        'title':'Home',
        'goto_form':goto_form
    }
    return render(request, 'radiotelescope/index.html', context=context)

def scan(request):
    context = {
        'title':'Scan'
    }
    return render(request, 'radiotelescope/scan.html', context=context)

def request_status(request):
    return HttpResponse(ratel.getStatus().__str__() + ' ' + round(ratel.getPosition().hr, 3).__str__() + ' ' + round(ratel.getPosition().dec, 3).__str__() + ' ' + ratel.getScanProgress().__str__() + ' ' + ratel.getSigStrength().__str__())

def command_goto(request):
    if request.method == "POST":
        dec = request.POST.get('dec')
        hr = request.POST.get('hr')
        ratel.sendGoto(Position(float(hr), float(dec)))
        return HttpResponseRedirect(reverse("radio_home"))
    return HttpResponseRedirect(reverse("radio_home"))

def command_stop(request):
    ratel.sendStop()
    return HttpResponse()

def command_home(request):
    ratel.sendHome()
    return HttpResponse()